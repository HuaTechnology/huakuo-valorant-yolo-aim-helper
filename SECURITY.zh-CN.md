<div align="right">

**简体中文** | [English](SECURITY.md)

</div>

# 安全策略

## 报告问题

发现安全问题时请**不要**开公开 issue，用 GitHub 的
[私密漏洞报告](https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper/security/advisories/new)，
或在 issue 里只写「需要私下沟通」并留联系方式。48 小时内回复。

## 本项目的攻击面

| 面 | 说明 | 建议 |
|---|---|---|
| 串口帧输入 | 固件解析来自主机的二进制帧，已做长度/CRC 校验与重同步 | 不要接入不可信设备 |
| 会话文件 | `out/telemetry/*.jsonl.gz` 含检测框坐标与时间戳，**可能间接暴露游戏画面信息** | 公开分享前先脱敏 |
| 配置/日志 | 可能包含本地路径、串口名、模型名 | 提交 issue 前先检查 |
| 模型文件 | `models/*.onnx` 由第三方提供，ONNX Runtime 解析不受信模型存在风险 | 只加载你有权使用且来源可信的模型 |

## 不要做的事

- 不要把 GitHub token、串口设备路径、内网地址提交进仓库（`.gitignore` 已覆盖常见产物）。
- 不要把 `configs/settings.json` 提交上去——它是本地生成的，仓库只保留 `settings.example.json`。
- 不要在 issue 里粘贴完整会话文件而不检查内容。

## 版本支持

只维护最新 release（当前 `v2.0.0`）。旧版本的问题请先升级再报告。
