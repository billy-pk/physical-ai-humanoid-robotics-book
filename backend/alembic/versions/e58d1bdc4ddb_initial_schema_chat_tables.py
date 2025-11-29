"""initial_schema_chat_tables

Revision ID: e58d1bdc4ddb
Revises: 
Create Date: 2025-11-29 12:30:27.597888

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = 'e58d1bdc4ddb'
down_revision: Union[str, Sequence[str], None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema - create chat tables."""
    # Create chat_sessions table
    op.create_table(
        'chat_sessions',
        sa.Column('session_id', sa.UUID(), nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.PrimaryKeyConstraint('session_id')
    )
    op.create_index('ix_chat_sessions_created_at', 'chat_sessions', ['created_at'])

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('message_id', sa.UUID(), nullable=False),
        sa.Column('session_id', sa.UUID(), nullable=False),
        sa.Column('role', sa.String(20), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('citations', sa.JSON(), nullable=True),
        sa.Column('highlighted_context', sa.Text(), nullable=True),
        sa.Column('tokens_used', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.PrimaryKeyConstraint('message_id'),
        sa.ForeignKeyConstraint(['session_id'], ['chat_sessions.session_id'], ondelete='CASCADE')
    )
    op.create_index('ix_chat_messages_session_id', 'chat_messages', ['session_id'])
    op.create_index('ix_chat_messages_created_at', 'chat_messages', ['created_at'])

    # Create api_metrics table
    op.create_table(
        'api_metrics',
        sa.Column('metric_id', sa.Integer(), autoincrement=True, nullable=False),
        sa.Column('endpoint', sa.String(255), nullable=False),
        sa.Column('method', sa.String(10), nullable=False),
        sa.Column('status_code', sa.Integer(), nullable=False),
        sa.Column('response_time_ms', sa.Float(), nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.PrimaryKeyConstraint('metric_id')
    )
    op.create_index('ix_api_metrics_endpoint', 'api_metrics', ['endpoint'])
    op.create_index('ix_api_metrics_created_at', 'api_metrics', ['created_at'])


def downgrade() -> None:
    """Downgrade schema - drop chat tables."""
    op.drop_index('ix_api_metrics_created_at', table_name='api_metrics')
    op.drop_index('ix_api_metrics_endpoint', table_name='api_metrics')
    op.drop_table('api_metrics')

    op.drop_index('ix_chat_messages_created_at', table_name='chat_messages')
    op.drop_index('ix_chat_messages_session_id', table_name='chat_messages')
    op.drop_table('chat_messages')

    op.drop_index('ix_chat_sessions_created_at', table_name='chat_sessions')
    op.drop_table('chat_sessions')
