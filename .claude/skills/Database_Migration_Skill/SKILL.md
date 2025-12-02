---
name: "database-migration"
description: "Create safe, reversible database migrations for schema changes. Handles adding tables, columns, indexes, and constraints without data loss. Use when evolving database schema."
version: "1.0.0"
---

# Database Migration Skill

## When to Use This Skill

- Adding new tables to the database
- Adding/removing columns from existing tables
- Creating or dropping indexes
- Modifying constraints (foreign keys, unique, check)
- Renaming tables or columns
- Changing column data types
- Deploying schema changes to production

## How This Skill Works

1. **Analyze Change**: Understand what schema modification is needed
2. **Write Up Migration**: Create SQL script to apply changes
3. **Write Down Migration**: Create SQL script to reverse changes
4. **Test Migration**: Verify up and down migrations work correctly
5. **Version Migration**: Assign sequential version number
6. **Document Migration**: Add comments explaining the change

## Migration File Structure

### Naming Convention
```
migrations/
├── 001_initial_schema.sql
├── 002_add_user_profiles.sql
├── 003_add_reading_progress.sql
├── 004_add_personalization_cache.sql
└── 005_add_indexes_for_performance.sql
```

Format: `{version}_{description}.sql`

### Template Structure

```sql
-- Migration: {Description}
-- Version: {number}
-- Created: {YYYY-MM-DD}
-- Author: {name}

BEGIN;

-- ==================================================
-- UP MIGRATION: Apply Changes
-- ==================================================

{SQL statements to apply changes}

COMMIT;
```

### Corresponding Down Migration

```sql
-- Migration Rollback: {Description}
-- Version: {number}
-- Created: {YYYY-MM-DD}

BEGIN;

-- ==================================================
-- DOWN MIGRATION: Revert Changes
-- ==================================================

{SQL statements to revert changes}

COMMIT;
```

## Example Migrations

### 1. Adding User Profiles Table

**Up Migration** (`002_add_user_profiles.sql`):
```sql
-- Migration: Add user profiles with background information
-- Version: 002
-- Created: 2025-01-15

BEGIN;

CREATE TABLE user_profiles (
    id SERIAL PRIMARY KEY,
    user_id INTEGER UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    software_background TEXT,
    hardware_background TEXT,
    experience_level VARCHAR(50) DEFAULT 'beginner' CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),
    preferred_topics TEXT[],
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_user_profiles_experience_level ON user_profiles(experience_level);

-- Grant permissions (if using RLS)
GRANT SELECT, INSERT, UPDATE ON user_profiles TO authenticated_user;

COMMIT;
```

**Down Migration** (`002_add_user_profiles_down.sql`):
```sql
-- Migration Rollback: Remove user profiles table
-- Version: 002
-- Created: 2025-01-15

BEGIN;

DROP TABLE IF EXISTS user_profiles CASCADE;

COMMIT;
```

### 2. Adding Reading Progress Tracking

**Up Migration** (`003_add_reading_progress.sql`):
```sql
-- Migration: Add reading progress tracking
-- Version: 003
-- Created: 2025-01-15

BEGIN;

CREATE TABLE reading_progress (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    chapter_id INTEGER REFERENCES chapters(id) ON DELETE CASCADE,
    progress_percentage INTEGER DEFAULT 0 CHECK (progress_percentage BETWEEN 0 AND 100),
    last_position JSONB,  -- {"section": 3, "scroll": 0.5}
    completed_at TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id, chapter_id)
);

CREATE INDEX idx_reading_progress_user ON reading_progress(user_id);
CREATE INDEX idx_reading_progress_chapter ON reading_progress(chapter_id);
CREATE INDEX idx_reading_progress_completed ON reading_progress(completed_at) WHERE completed_at IS NOT NULL;

COMMIT;
```

**Down Migration**:
```sql
BEGIN;
DROP TABLE IF EXISTS reading_progress CASCADE;
COMMIT;
```

### 3. Adding Column to Existing Table

**Up Migration** (`004_add_avatar_to_users.sql`):
```sql
-- Migration: Add avatar URL to users table
-- Version: 004
-- Created: 2025-01-16

BEGIN;

ALTER TABLE users ADD COLUMN avatar_url VARCHAR(500);
ALTER TABLE users ADD COLUMN last_login_at TIMESTAMP;

-- Create index for last login queries
CREATE INDEX idx_users_last_login ON users(last_login_at DESC) WHERE last_login_at IS NOT NULL;

COMMIT;
```

**Down Migration**:
```sql
BEGIN;

-- Drop indexes first
DROP INDEX IF EXISTS idx_users_last_login;

-- Drop columns
ALTER TABLE users DROP COLUMN IF EXISTS avatar_url;
ALTER TABLE users DROP COLUMN IF EXISTS last_login_at;

COMMIT;
```

### 4. Modifying Column Data Type

**Up Migration** (`005_expand_email_length.sql`):
```sql
-- Migration: Expand email column length
-- Version: 005
-- Created: 2025-01-17

BEGIN;

-- Check if any existing emails would be truncated (safety check)
DO $$
BEGIN
    IF EXISTS (SELECT 1 FROM users WHERE LENGTH(email) > 255) THEN
        RAISE EXCEPTION 'Cannot modify column: emails longer than 255 chars exist';
    END IF;
END $$;

-- Modify column type
ALTER TABLE users ALTER COLUMN email TYPE VARCHAR(320);

COMMIT;
```

**Down Migration**:
```sql
BEGIN;

-- Verify data fits in smaller size
DO $$
BEGIN
    IF EXISTS (SELECT 1 FROM users WHERE LENGTH(email) > 255) THEN
        RAISE EXCEPTION 'Cannot revert: emails longer than 255 chars exist';
    END IF;
END $$;

ALTER TABLE users ALTER COLUMN email TYPE VARCHAR(255);

COMMIT;
```

### 5. Adding Indexes for Performance

**Up Migration** (`006_add_performance_indexes.sql`):
```sql
-- Migration: Add indexes to improve query performance
-- Version: 006
-- Created: 2025-01-18

BEGIN;

-- Index for chatbot conversation lookups
CREATE INDEX idx_chatbot_messages_conversation ON chatbot_messages(conversation_id);
CREATE INDEX idx_chatbot_messages_created ON chatbot_messages(created_at DESC);

-- Index for personalized content cache
CREATE INDEX idx_personalized_user_chapter ON personalized_content(user_id, chapter_id);
CREATE INDEX idx_personalized_expires ON personalized_content(expires_at);

-- Partial index for active sessions only
CREATE INDEX idx_active_sessions ON sessions(user_id) WHERE expires_at > NOW();

COMMIT;
```

**Down Migration**:
```sql
BEGIN;

DROP INDEX IF EXISTS idx_chatbot_messages_conversation;
DROP INDEX IF EXISTS idx_chatbot_messages_created;
DROP INDEX IF EXISTS idx_personalized_user_chapter;
DROP INDEX IF EXISTS idx_personalized_expires;
DROP INDEX IF EXISTS idx_active_sessions;

COMMIT;
```

## Migration Runner Script

### Python Implementation

```python
import asyncpg
import os
from pathlib import Path
from typing import List

MIGRATIONS_DIR = Path("migrations")

async def get_applied_migrations(conn) -> List[str]:
    """Get list of already applied migrations from database."""
    await conn.execute("""
        CREATE TABLE IF NOT EXISTS schema_migrations (
            version VARCHAR(50) PRIMARY KEY,
            applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)

    rows = await conn.fetch("SELECT version FROM schema_migrations ORDER BY version")
    return [row['version'] for row in rows]

async def apply_migration(conn, migration_file: Path):
    """Apply a single migration file."""
    version = migration_file.stem  # e.g., "001_initial_schema"

    print(f"Applying migration: {version}")

    with open(migration_file, 'r') as f:
        sql = f.read()

    try:
        await conn.execute(sql)
        await conn.execute(
            "INSERT INTO schema_migrations (version) VALUES ($1)",
            version
        )
        print(f"✓ Applied: {version}")
    except Exception as e:
        print(f"✗ Failed: {version}")
        raise

async def rollback_migration(conn, migration_file: Path):
    """Rollback a single migration."""
    version = migration_file.stem

    # Find corresponding down migration
    down_file = migration_file.parent / f"{version}_down.sql"

    if not down_file.exists():
        raise FileNotFoundError(f"Down migration not found: {down_file}")

    print(f"Rolling back migration: {version}")

    with open(down_file, 'r') as f:
        sql = f.read()

    try:
        await conn.execute(sql)
        await conn.execute(
            "DELETE FROM schema_migrations WHERE version = $1",
            version
        )
        print(f"✓ Rolled back: {version}")
    except Exception as e:
        print(f"✗ Rollback failed: {version}")
        raise

async def migrate_up(database_url: str):
    """Apply all pending migrations."""
    conn = await asyncpg.connect(database_url)

    try:
        applied = await get_applied_migrations(conn)

        # Find all migration files (excluding _down.sql)
        migration_files = sorted([
            f for f in MIGRATIONS_DIR.glob("*.sql")
            if not f.name.endswith("_down.sql")
        ])

        pending = [
            f for f in migration_files
            if f.stem not in applied
        ]

        if not pending:
            print("No pending migrations.")
            return

        print(f"Found {len(pending)} pending migration(s)")

        for migration_file in pending:
            await apply_migration(conn, migration_file)

        print("All migrations applied successfully!")

    finally:
        await conn.close()

async def migrate_down(database_url: str, steps: int = 1):
    """Rollback the last N migrations."""
    conn = await asyncpg.connect(database_url)

    try:
        applied = await get_applied_migrations(conn)

        if not applied:
            print("No migrations to rollback.")
            return

        to_rollback = applied[-steps:]

        for version in reversed(to_rollback):
            migration_file = MIGRATIONS_DIR / f"{version}.sql"
            await rollback_migration(conn, migration_file)

        print(f"Rolled back {len(to_rollback)} migration(s)")

    finally:
        await conn.close()

# Usage
import asyncio

if __name__ == "__main__":
    import sys

    db_url = os.getenv("DATABASE_URL")

    if len(sys.argv) > 1 and sys.argv[1] == "down":
        steps = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        asyncio.run(migrate_down(db_url, steps))
    else:
        asyncio.run(migrate_up(db_url))
```

### Usage

```bash
# Apply all pending migrations
python migrate.py

# Rollback last migration
python migrate.py down 1

# Rollback last 3 migrations
python migrate.py down 3
```

## Migration Best Practices

### Safety Checklist

- ✓ **Always use transactions** (BEGIN/COMMIT)
- ✓ **Create down migrations** for every up migration
- ✓ **Test migrations** on a copy of production data
- ✓ **Add safety checks** before destructive operations
- ✓ **Backup database** before running migrations in production
- ✓ **Version migrations** sequentially
- ✓ **Document changes** with clear comments

### Common Pitfalls to Avoid

- ❌ **Don't modify applied migrations** - create new ones instead
- ❌ **Don't use DROP COLUMN** without checking data first
- ❌ **Don't change column types** without verifying data compatibility
- ❌ **Don't skip version numbers** - keep sequential ordering
- ❌ **Don't run migrations without backups** in production

### Data Migration Pattern

When you need to migrate data along with schema changes:

```sql
-- Migration: Convert user role from string to enum
-- Version: 007

BEGIN;

-- Step 1: Add new column
ALTER TABLE users ADD COLUMN role_new VARCHAR(50);

-- Step 2: Migrate data
UPDATE users SET role_new = CASE
    WHEN role = 'admin' THEN 'administrator'
    WHEN role = 'user' THEN 'regular_user'
    ELSE 'regular_user'
END;

-- Step 3: Add constraints
ALTER TABLE users ALTER COLUMN role_new SET NOT NULL;
ALTER TABLE users ADD CONSTRAINT check_role CHECK (role_new IN ('administrator', 'regular_user', 'moderator'));

-- Step 4: Drop old column and rename new
ALTER TABLE users DROP COLUMN role;
ALTER TABLE users RENAME COLUMN role_new TO role;

COMMIT;
```

## Testing Migrations

### Test Script

```python
async def test_migration():
    # Connect to test database
    conn = await asyncpg.connect("postgresql://localhost/test_db")

    try:
        # Apply migration
        await apply_migration(conn, Path("migrations/002_add_user_profiles.sql"))

        # Verify table exists
        exists = await conn.fetchval("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'user_profiles'
            )
        """)
        assert exists, "Table was not created"

        # Verify columns
        columns = await conn.fetch("""
            SELECT column_name FROM information_schema.columns
            WHERE table_name = 'user_profiles'
        """)
        column_names = [c['column_name'] for c in columns]
        assert 'user_id' in column_names
        assert 'software_background' in column_names

        # Test rollback
        await rollback_migration(conn, Path("migrations/002_add_user_profiles.sql"))

        # Verify table is dropped
        exists = await conn.fetchval("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'user_profiles'
            )
        """)
        assert not exists, "Table was not dropped"

        print("✓ Migration test passed!")

    finally:
        await conn.close()
```

## Output Format

When creating a migration, provide:

1. **Migration Number and Description**:
   - Sequential version number
   - Clear description of what changes

2. **Up Migration SQL**:
   - All SQL statements to apply changes
   - Safety checks for data validation
   - Comments explaining each step

3. **Down Migration SQL**:
   - All SQL statements to reverse changes
   - Order reversed from up migration

4. **Testing Notes**:
   - How to verify migration succeeded
   - Expected table/column state after migration

5. **Deployment Instructions**:
   - Any special considerations for production
   - Required backups or preparations
