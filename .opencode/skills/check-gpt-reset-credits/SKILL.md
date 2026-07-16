---
name: check-gpt-reset-credits
description: 查询 ChatGPT Plus 账户的 rate-limit reset credits（重置卡）剩余数量、过期时间等信息。source: ~/.codex/auth.json, endpoint: /backend-api/wham/rate-limit-reset-credits
---

# Check GPT Rate-Limit Reset Credits

查询本机 Codex 凭证关联的 ChatGPT 账户还剩多少 rate-limit reset credits "重置卡"。

## 触发词

- "查一下重置卡"
- "还有多少重置卡"
- "rate-limit reset credits"
- "重置次数"
- "gpt 重置卡"

## 流程

### 1. 读取 access_token

```bash
TOKEN=$(python3 -c "import json; print(json.load(open('/home/minzhi/.codex/auth.json'))['tokens']['access_token'])")
```

如果文件不存在或缺少 `access_token`，报错并提示用户检查 `~/.codex/auth.json`。

### 2. 请求 API

```bash
curl -s -o /tmp/rate_limit_resp.json -w "%{http_code}" \
  -H "Authorization: Bearer $TOKEN" \
  "https://chatgpt.com/backend-api/wham/rate-limit-reset-credits"
```

### 3. 解析并渲染结果

用 Python 解析 `/tmp/rate_limit_resp.json`，要求：

- **隐藏敏感信息**：不打印 `access_token`、`refresh_token`、`account_id`、`user_id` 等任何唯一标识和 cookie。
- **输出格式**：

  ```
  available_count: 4

    [1] status:     available
        title:     Full reset
        granted:   2026-06-18 08:37:47 CST
        expires:   2026-07-18 08:37:47 CST
  ...
  ```

- **granted_at / expires_at**：原始值为 UTC ISO8601，统一转为 **CST (UTC+8)** 显示。
- **列表为空**：输出 `No credits found.`

### 4. 错误处理

| HTTP 状态码 | 含义 | 处理方式 |
|---|---|---|
| 200 | 成功 | 正常渲染 |
| 401 | 凭证失效 | 输出: `401 Unauthorized — access_token 已过期或无效。请重新登录 Codex 以刷新凭证。` |
| 其他 | 未知错误 | 输出: `HTTP <code> — 请求失败，响应体: <body>` |

## 示例

用户说 "查一下重置卡"，返回：

```
available_count: 4

  [1] status:     available
      title:     Full reset
      granted:   2026-06-18 08:37:47 CST
      expires:   2026-07-18 08:37:47 CST

  [2] status:     available
      title:     Full reset
      granted:   2026-06-27 08:10:28 CST
      expires:   2026-07-27 08:10:28 CST

  [3] status:     available
      title:     Full reset
      granted:   2026-07-02 04:35:05 CST
      expires:   2026-08-01 04:35:05 CST

  [4] status:     available
      title:     Full reset
      granted:   2026-07-14 02:07:55 CST
      expires:   2026-08-13 02:07:55 CST
```

## 安全注意事项

- 任何时候都不要在回复中输出 `access_token`、`refresh_token`、`account_id`、`user_id`、`session_id`、`jti` 等唯一标识字段。
- 使用 `python3 -c` 或临时文件提取 token，不要用 `echo` 泄漏到 shell 历史或日志。
