//! 词法：一行文本 → 字序列。
//!
//! 只做**切词**，不做语义。所有语义判断在 [`crate::interp`]。
//!
//! ## 处理的语法
//!
//! | 形式 | 处理 |
//! |---|---|
//! | `G1 X10.5` | 字 `(G, 1)`、`(X, 10.5)` |
//! | `(注释)` | 收进 `comment`，可出现在行内任意位置 |
//! | `;注释` | 行尾注释（Fanuc 风格） |
//! | `%` | 程序分隔符，整行忽略 |
//! | `N123` | 行号字，作为普通字保留 |
//! | `*xx` | 旧式校验和，`*` 之后全部丢弃 |
//!
//! ## v1 的一处刻意偏差
//!
//! LinuxCNC 允许在数字**中间**插入空白（`overview.adoc` §Use consistent white space）。
//! 我们**不**支持：空白一律当字分隔符，裸数字（前面没有字母）报 `GCODE-006`。
//! 理由是这个写法在真实程序里几乎不出现，而支持它会引入"`X1 -5` 到底是
//! `X1` 还是 `X1-5`"的歧义。报 issue 而不是静默吞掉，符合"不猜"的原则。

use crate::{ParseIssue, Severity};

/// 一个字：字母 + 数值。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Word {
    /// 大写字母的字节，如 `b'G'`
    pub letter: u8,
    pub value: f64,
    /// 数值原文里有没有小数点。
    ///
    /// **这不是细节**：Fanuc 的 `P`（停延）按"有没有小数点"解释单位——
    /// `P500` 是 500 **毫秒**，`P0.5` 是 0.5 **秒**。丢掉这个信息，
    /// 钻孔循环的孔底暂停会差 1000 倍。
    pub has_dot: bool,
}

impl Word {
    /// 构造一个字。`has_dot` 默认 `false`（源码里没写小数点）。
    pub fn new(letter: u8, value: f64) -> Self {
        Self {
            letter: letter.to_ascii_uppercase(),
            value,
            has_dot: false,
        }
    }

    /// 带小数点标记的构造。
    pub fn parsed(letter: u8, value: f64, has_dot: bool) -> Self {
        Self {
            has_dot,
            ..Self::new(letter, value)
        }
    }
    /// 是否为某个字母（不区分大小写）
    pub fn is(&self, letter: u8) -> bool {
        self.letter == letter.to_ascii_uppercase()
    }
}

/// 一行词法结果。
#[derive(Clone, Debug, Default)]
pub struct LexedLine {
    pub line_no: u32,
    pub words: Vec<Word>,
    /// 最后一个注释的内容（多个注释只留最后一个，同 LinuxCNC）
    pub comment: Option<String>,
    /// 空行 / 纯注释行 / 程序分隔符
    pub blank: bool,
}

impl LexedLine {
    /// 取第一个某个字母的字
    pub fn word(&self, letter: u8) -> Option<&Word> {
        self.words.iter().find(|w| w.is(letter))
    }
    /// 取所有某个字母的字
    pub fn words_of(&self, letter: u8) -> impl Iterator<Item = &Word> {
        let l = letter.to_ascii_uppercase();
        self.words.iter().filter(move |w| w.letter == l)
    }
    pub fn value_of(&self, letter: u8) -> Option<f64> {
        self.word(letter).map(|w| w.value)
    }
}

/// 把整份源码切成行。
pub fn lex(src: &str) -> (Vec<LexedLine>, Vec<ParseIssue>) {
    let mut out = Vec::new();
    let mut issues = Vec::new();
    for (i, raw) in src.lines().enumerate() {
        let line_no = (i + 1) as u32;
        let (line, mut e) = lex_line(raw, line_no);
        issues.append(&mut e);
        out.push(line);
    }
    (out, issues)
}

/// 切一行。
pub fn lex_line(raw: &str, line_no: u32) -> (LexedLine, Vec<ParseIssue>) {
    let mut words: Vec<Word> = Vec::new();
    let mut comment: Option<String> = None;
    let mut issues = Vec::new();
    let mut blank = true;

    let bytes: Vec<char> = raw.chars().collect();
    let mut i = 0usize;
    // 当前正在累积的字
    let mut cur_letter: Option<u8> = None;
    let mut num = String::new();

    // 把已累积的字收尾
    macro_rules! flush {
        () => {
            if let Some(letter) = cur_letter.take() {
                match num.parse::<f64>() {
                    Ok(v) if v.is_finite() => {
                        words.push(Word::parsed(letter, v, num.contains('.')))
                    }
                    _ => issues.push(ParseIssue::new(
                        "GCODE-006",
                        Severity::Error,
                        line_no,
                        format!("字 {letter} 的数值无法解析：{num:?}"),
                        "检查该字后面是否跟了合法数字（允许前导 +/- 与小数点）",
                    )),
                }
                num.clear();
            }
        };
    }

    while i < bytes.len() {
        let c = bytes[i];

        // 括号注释：不嵌套，遇到第一个 ')' 结束
        if c == '(' {
            let start = i + 1;
            let mut j = start;
            while j < bytes.len() && bytes[j] != ')' {
                j += 1;
            }
            let text: String = bytes[start..j.min(bytes.len())].iter().collect();
            comment = Some(text.trim().to_string());
            blank = false;
            i = if j < bytes.len() { j + 1 } else { j };
            continue;
        }

        // 行尾注释
        if c == ';' {
            let text: String = bytes[i + 1..].iter().collect();
            comment = Some(text.trim().to_string());
            blank = false;
            break;
        }

        // 旧式校验和：'*' 之后全部丢弃
        if c == '*' {
            break;
        }

        // 程序分隔符：整行忽略（LinuxCNC 里 % 只出现在首尾）
        if c == '%' {
            flush!();
            return (
                LexedLine {
                    line_no,
                    words: Vec::new(),
                    comment: None,
                    blank: true,
                },
                issues,
            );
        }

        // 字母：开始一个新字
        if c.is_ascii_alphabetic() {
            flush!();
            cur_letter = Some(c.to_ascii_uppercase() as u8);
            blank = false;
            i += 1;
            continue;
        }

        // 数字、符号、小数点：累积到当前字
        if c.is_ascii_digit() || c == '.' || c == '+' || c == '-' {
            if cur_letter.is_none() {
                // 裸数字——见模块文档的"刻意偏差"
                let mut j = i;
                let mut s = String::new();
                while j < bytes.len()
                    && (bytes[j].is_ascii_digit() || bytes[j] == '.' || bytes[j] == '+' || bytes[j] == '-')
                {
                    s.push(bytes[j]);
                    j += 1;
                }
                issues.push(ParseIssue::new(
                    "GCODE-006",
                    Severity::Error,
                    line_no,
                    format!("数字 {s:?} 前面没有字母"),
                    "G 代码是「字母+数值」格式，如 X10；检查是否漏了轴名，或把空格写进了数字中间",
                ));
                i = j;
                continue;
            }
            num.push(c);
            blank = false;
            i += 1;
            continue;
        }

        // 空白：**终止当前字**。
        //
        // LinuxCNC 的 Best Practices 说"允许在数字中间插空白"，但那条一旦照做，
        // `G1 10` 会被静默拼成 `G110`——一个字的错误变成另一个合法字的错误，
        // 比报错难查得多。所以这里**终止**字，裸数字走 GCODE-006 报出来。
        if c.is_whitespace() {
            flush!();
            i += 1;
            continue;
        }

        // 其它字符：跳过
        i += 1;
    }
    flush!();

    let is_blank = blank || (words.is_empty() && comment.is_none());

    (
        LexedLine {
            line_no,
            words,
            comment,
            blank: is_blank,
        },
        issues,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_words() {
        let (l, e) = lex_line("G1 X10.5 Y-3", 1);
        assert!(e.is_empty());
        assert_eq!(
            l.words,
            vec![
                Word::new(b'G', 1.0),
                Word::parsed(b'X', 10.5, true),
                Word::new(b'Y', -3.0),
            ]
        );
    }

    #[test]
    fn decimal_point_is_recorded() {
        // P500 是 500 毫秒、P0.5 是 0.5 秒 —— 单位由小数点决定，必须留住
        let (l, _) = lex_line("P500 P0.5", 1);
        assert!(!l.words[0].has_dot);
        assert!(l.words[1].has_dot);
    }

    #[test]
    fn lowercase_is_normalized() {
        let (l, _) = lex_line("g1 x10", 1);
        assert_eq!(l.words[0], Word::new(b'G', 1.0));
        assert_eq!(l.words[1], Word::new(b'X', 10.0));
    }

    #[test]
    fn comments_are_extracted() {
        let (l, _) = lex_line("G1 (rapid) X10 (to start)", 1);
        assert_eq!(l.comment.as_deref(), Some("to start"), "多个注释只留最后一个");
        assert_eq!(l.words.len(), 2);
    }

    #[test]
    fn semicolon_comment_runs_to_end() {
        let (l, _) = lex_line("G0 X1 ; move away X9", 1);
        assert_eq!(l.comment.as_deref(), Some("move away X9"));
        assert_eq!(l.words.len(), 2, "注释里的 X9 不能被当成字");
    }

    #[test]
    fn percent_line_is_blank() {
        let (l, _) = lex_line("%", 1);
        assert!(l.blank);
        assert!(l.words.is_empty());
    }

    #[test]
    fn checksum_is_dropped() {
        let (l, _) = lex_line("N1 G1 X1*57", 1);
        assert_eq!(l.words.len(), 3);
        assert_eq!(l.words[2], Word::new(b'X', 1.0));
    }

    #[test]
    fn bare_number_is_reported() {
        // 刻意偏差：不支持"数字中间插空白"，裸数字要报错而不是静默吞
        let (_, e) = lex_line("X1 0", 1);
        assert_eq!(e.len(), 1);
        assert_eq!(e[0].id, "GCODE-006");
        assert!(e[0].hint.is_some());
    }

    #[test]
    fn repeated_letter_keeps_all_occurrences() {
        let (l, _) = lex_line("G1 G0 X1", 1);
        assert_eq!(l.words_of(b'G').count(), 2, "同字母多字都要保留，交由语义层判冲突");
    }
}
