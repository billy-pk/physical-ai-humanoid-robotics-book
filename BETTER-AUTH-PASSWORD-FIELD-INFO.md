# Better Auth Password Field Information

## Issue: Sign-in Failing After Signup

You're experiencing sign-in failures after user creation. Better Auth expects passwords to be stored in a specific table and field.

## Better Auth Password Storage

### Table: `account` (NOT `user`)

Better Auth stores passwords in the **`account`** table, not the `user` table. The `user` table only stores user profile information (id, name, email, etc.).

### Password Field Name

According to Better Auth documentation, the password is stored in the `account` table with the field name:
- **`password`** (text field)

### Account Table Schema

The `account` table should have the following structure:

```sql
CREATE TABLE account (
    id TEXT PRIMARY KEY,
    accountId TEXT NOT NULL,
    providerId TEXT NOT NULL,  -- 'credential' for email/password
    userId TEXT NOT NULL,       -- Foreign key to user.id
    password TEXT,              -- ◄── PASSWORD STORED HERE
    accessToken TEXT,
    refreshToken TEXT,
    idToken TEXT,
    accessTokenExpiresAt TIMESTAMP,
    refreshTokenExpiresAt TIMESTAMP,
    scope TEXT,
    createdAt TIMESTAMP NOT NULL,
    updatedAt TIMESTAMP NOT NULL,
    FOREIGN KEY (userId) REFERENCES user(id) ON DELETE CASCADE
);
```

### Key Points

1. **Table**: `account` (not `user`)
2. **Field**: `password` (text type)
3. **Provider**: For email/password auth, `providerId` should be `'credential'`
4. **Relationship**: `account.userId` → `user.id`

## Checking Your Database

Run this query to check if the account table exists and has the password field:

```sql
-- Check if account table exists
SELECT table_name 
FROM information_schema.tables 
WHERE table_schema = 'public' 
AND table_name = 'account';

-- Check account table columns
SELECT column_name, data_type 
FROM information_schema.columns 
WHERE table_name = 'account' 
ORDER BY ordinal_position;

-- Check if password field exists
SELECT column_name 
FROM information_schema.columns 
WHERE table_name = 'account' 
AND column_name = 'password';
```

## Troubleshooting

### If `account` table doesn't exist:

Better Auth should create this table automatically when you first use email/password authentication. If it doesn't exist:

1. **Check Better Auth logs** for migration errors
2. **Verify database connection** in Better Auth config
3. **Check permissions** - Better Auth needs CREATE TABLE permissions

### If `password` field doesn't exist:

1. **Check Better Auth version** - Ensure you're using a compatible version
2. **Verify emailAndPassword is enabled** in Better Auth config
3. **Check for custom field mappings** - If you customized field names, ensure password is mapped correctly

### If sign-in still fails:

1. **Check account record exists** after signup:
   ```sql
   SELECT * FROM account WHERE userId = 'your-user-id';
   ```

2. **Verify providerId** is `'credential'` for email/password:
   ```sql
   SELECT providerId FROM account WHERE userId = 'your-user-id';
   ```

3. **Check password is hashed** (should not be plain text):
   ```sql
   SELECT password FROM account WHERE userId = 'your-user-id';
   ```

## Better Auth Configuration

Your current config in `backend/auth-service/src/auth.ts`:

```typescript
emailAndPassword: {
  enabled: true,
  requireEmailVerification: false,
  minPasswordLength: 8,
  maxPasswordLength: 128,
  autoSignIn: true,
}
```

This looks correct. Better Auth should automatically:
1. Create the `account` table if it doesn't exist
2. Store hashed passwords in `account.password`
3. Use `providerId = 'credential'` for email/password accounts

## Next Steps

1. **Verify account table exists** in your database
2. **Check if password field exists** in account table
3. **Verify account records are created** after signup
4. **Check Better Auth logs** for any errors during signup/signin

