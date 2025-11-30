"""add_user_id_to_chat_sessions

Revision ID: b20f7b5c7f26
Revises: e58d1bdc4ddb
Create Date: 2025-11-30 01:26:34.941553

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = 'b20f7b5c7f26'
down_revision: Union[str, Sequence[str], None] = 'e58d1bdc4ddb'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Add user_id column to chat_sessions table to link sessions to authenticated users."""
    # Add user_id column (text to match Better Auth user.id type, nullable for anonymous sessions)
    op.add_column('chat_sessions', sa.Column('user_id', sa.Text(), nullable=True))
    
    # Add foreign key constraint to user table (created by Better Auth)
    # Note: Better Auth creates 'user' table (singular), not 'users'
    op.create_foreign_key(
        'fk_chat_sessions_user_id',
        'chat_sessions',
        'user',
        ['user_id'],
        ['id'],
        ondelete='SET NULL'  # Set to NULL if user is deleted, preserve chat history
    )
    
    # Add index on user_id for faster queries
    op.create_index('ix_chat_sessions_user_id', 'chat_sessions', ['user_id'])


def downgrade() -> None:
    """Remove user_id column from chat_sessions table."""
    op.drop_index('ix_chat_sessions_user_id', table_name='chat_sessions')
    op.drop_constraint('fk_chat_sessions_user_id', 'chat_sessions', type_='foreignkey')
    op.drop_column('chat_sessions', 'user_id')
