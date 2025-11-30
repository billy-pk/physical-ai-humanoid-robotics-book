"""create_user_background_questionnaire

Revision ID: 954501217eb6
Revises: 21c54f44e3e4
Create Date: 2025-11-30 01:26:46.819004

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '954501217eb6'
down_revision: Union[str, Sequence[str], None] = '21c54f44e3e4'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Create user_background_questionnaire table for tracking individual questionnaire answers."""
    op.create_table(
        'user_background_questionnaire',
        sa.Column('id', sa.UUID(), nullable=False),
        sa.Column('user_id', sa.Text(), nullable=False),  # Text to match Better Auth user.id
        sa.Column('question_id', sa.String(100), nullable=False),  # Identifier for the question
        sa.Column('answer', sa.JSON(), nullable=True),  # Flexible JSON answer format
        sa.Column('answered_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.ForeignKeyConstraint(['user_id'], ['user.id'], ondelete='CASCADE')
    )
    
    # Create indexes for efficient queries
    op.create_index('ix_user_background_questionnaire_user_id', 'user_background_questionnaire', ['user_id'])
    op.create_index('ix_user_background_questionnaire_question_id', 'user_background_questionnaire', ['question_id'])


def downgrade() -> None:
    """Drop user_background_questionnaire table."""
    op.drop_index('ix_user_background_questionnaire_question_id', table_name='user_background_questionnaire')
    op.drop_index('ix_user_background_questionnaire_user_id', table_name='user_background_questionnaire')
    op.drop_table('user_background_questionnaire')
