# How to View FastAPI Logs

## Quick Answer

FastAPI logs are displayed in **the terminal where FastAPI is running**. They appear in real-time as requests are made.

## Where to Find Logs

### 1. **Terminal Where FastAPI is Running**

FastAPI logs are output to the console (stdout/stderr). Look at the terminal where you started FastAPI:

```bash
# Terminal 2: FastAPI Backend (this is where logs appear)
cd backend
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

All log messages will appear in this terminal, including:
- Request logs
- Authentication validation logs
- Database query logs
- Error messages

### 2. **What Logs Look Like**

FastAPI uses structured logging (structlog), so logs appear like:

```
2025-12-02T07:45:19.720Z [info] get_user_from_session called with token: fZpLdNpt57mmHZwHbChd...
2025-12-02T07:45:19.721Z [info] Executing session query with token: fZpLdNpt57mmHZwHbChd...
2025-12-02T07:45:19.722Z [info] Session validated for user: XjoWLjCn14r8wWbOWhCCgsC1zGPFpe6v, email: billypk735@gmail.com
```

### 3. **Key Log Messages to Look For**

When debugging the 401 error, look for these messages:

- `get_current_active_user - Authorization header: ...` - Shows if token is received
- `get_user_from_session called with token: ...` - Shows token being validated
- `Executing session query with token: ...` - Shows database query starting
- `Session validated for user: ...` - Success! Session is valid
- `Session not found or expired` - Token doesn't exist or expired
- `Error validating session from database: ...` - Database query failed

## How to View Logs

### Option 1: Watch the Terminal
Simply keep the FastAPI terminal visible and watch for log messages when you make requests.

### Option 2: Save Logs to File
If you want to save logs to a file, redirect output:

```bash
cd backend
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000 2>&1 | tee fastapi.log
```

This will:
- Display logs in terminal (real-time)
- Save logs to `fastapi.log` file

### Option 3: Use `tail` to Follow Logs
If you saved logs to a file:

```bash
tail -f backend/fastapi.log
```

## Debugging the 401 Error

When you sign in and get a 401 error, check the FastAPI terminal for:

1. **Token Received?**
   ```
   get_current_active_user - Authorization header: Bearer fZpLdNpt57mmHZwHbChd...
   ```

2. **Token Extracted?**
   ```
   get_current_active_user - Extracted Bearer token: fZpLdNpt57mmHZwHbChd...
   ```

3. **Database Query?**
   ```
   Executing session query with token: fZpLdNpt57mmHZwHbChd...
   ```

4. **Result?**
   - Success: `Session validated for user: ...`
   - Failure: `Session not found or expired` or `Session token not found in database`

## Increase Log Level (More Details)

To see more detailed logs, you can change the log level in `backend/src/core/logging.py`:

```python
logging.basicConfig(format="%(message)s", level=logging.DEBUG)  # Changed from INFO to DEBUG
```

Or when starting uvicorn:

```bash
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000 --log-level debug
```

## Quick Test

To test if logging is working, make a request and watch the terminal:

```bash
# In another terminal
curl -X GET "http://localhost:8000/api/auth/profile/preferences" \
  -H "Authorization: Bearer YOUR_TOKEN_HERE" \
  -H "Content-Type: application/json"
```

You should see log messages appear in the FastAPI terminal immediately.

