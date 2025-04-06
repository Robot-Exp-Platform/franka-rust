# Change log (更新日志)

## v0.1.5 (2025-04-06)

- Fix: 修复了 linux 下对于 model 的重力和科里奥利力的问题

- Error: Windows 下 udp 发送的指令无法被接收到，在 20 帧之后会导致 `CommunicationConstraintsViolation` 错误

## v0.1.3 (2025-03-29)

- Feat: model 的下载和加载功能（模型的下载会导致至少 4ms 的延迟）在执行运动中下载可能会导致问题。

## v0.1.0 (2025-03-26)

- Initial release (初始版本)
