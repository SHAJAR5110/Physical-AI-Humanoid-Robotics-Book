"""Create initial database tables: users, sessions, chapters, embeddings, personalized_content, chat_messages

Revision ID: 001
Revises:
Create Date: 2025-12-06 12:00:00.000000
"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = '001'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    """Create all initial tables."""

    # Create users table
    op.create_table(
        'users',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('email', sa.String(255), nullable=False),
        sa.Column('name', sa.String(255), nullable=False),
        sa.Column('password_hash', sa.String(255), nullable=False),
        sa.Column('os', sa.String(50), nullable=True),
        sa.Column('gpu', sa.String(100), nullable=True),
        sa.Column('experience_level', sa.String(50), nullable=True),
        sa.Column('robotics_background', sa.Boolean(), nullable=False, server_default=sa.false()),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), onupdate=sa.func.now()),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('email'),
    )
    op.create_index('idx_users_email', 'users', ['email'], unique=True)
    op.create_index('idx_users_created_at', 'users', ['created_at'])

    # Create sessions table
    op.create_table(
        'sessions',
        sa.Column('session_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('token_hash', sa.String(255), nullable=False),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('session_id'),
        sa.UniqueConstraint('token_hash'),
    )
    op.create_index('idx_sessions_user_id', 'sessions', ['user_id'])
    op.create_index('idx_sessions_token_hash', 'sessions', ['token_hash'], unique=True)
    op.create_index('idx_sessions_expires_at', 'sessions', ['expires_at'])

    # Create chapters table
    op.create_table(
        'chapters',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(255), nullable=False),
        sa.Column('slug', sa.String(255), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('summary', sa.Text(), nullable=True),
        sa.Column('author', sa.String(255), nullable=True),
        sa.Column('version', sa.String(10), nullable=False, server_default='1.0'),
        sa.Column('page_count', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), onupdate=sa.func.now()),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('slug'),
    )
    op.create_index('idx_chapters_slug', 'chapters', ['slug'], unique=True)
    op.create_index('idx_chapters_created_at', 'chapters', ['created_at'])

    # Create chapter_embeddings table
    op.create_table(
        'chapter_embeddings',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('section_title', sa.String(255), nullable=True),
        sa.Column('section_index', sa.Integer(), nullable=True),
        sa.Column('content_excerpt', sa.Text(), nullable=False),
        sa.Column('embedding_model', sa.String(50), nullable=False, server_default='claude-embeddings-3'),
        sa.Column('vector_dimensions', sa.Integer(), nullable=False, server_default='1024'),
        sa.Column('qdrant_point_id', postgresql.UUID(as_uuid=True), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), onupdate=sa.func.now()),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('id'),
    )
    op.create_index('idx_chapter_embeddings_chapter_id', 'chapter_embeddings', ['chapter_id'])
    op.create_index('idx_chapter_embeddings_qdrant_point_id', 'chapter_embeddings', ['qdrant_point_id'])

    # Create personalized_content table
    op.create_table(
        'personalized_content',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('personalized_text', sa.Text(), nullable=False),
        sa.Column('model_version', sa.String(50), nullable=False, server_default='claude-3-5-haiku-20241022'),
        sa.Column('cache_source', sa.String(50), nullable=False, server_default='claude-api'),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.Column('expires_at', sa.DateTime(timezone=True), nullable=False),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ondelete='CASCADE'),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('user_id', 'chapter_id'),
    )
    op.create_index('idx_personalized_content_user_id', 'personalized_content', ['user_id'])
    op.create_index('idx_personalized_content_chapter_id', 'personalized_content', ['chapter_id'])
    op.create_index('idx_personalized_content_user_chapter', 'personalized_content', ['user_id', 'chapter_id'])
    op.create_index('idx_personalized_content_expires_at', 'personalized_content', ['expires_at'])

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('user_message', sa.Text(), nullable=False),
        sa.Column('assistant_response', sa.Text(), nullable=False),
        sa.Column('citations', sa.Text(), nullable=True),
        sa.Column('source_chapters', sa.Text(), nullable=True),
        sa.Column('tokens_used', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=False),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ondelete='CASCADE'),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('id'),
    )
    op.create_index('idx_chat_messages_user_id', 'chat_messages', ['user_id'])
    op.create_index('idx_chat_messages_chapter_id', 'chat_messages', ['chapter_id'])
    op.create_index('idx_chat_messages_created_at', 'chat_messages', ['created_at'])


def downgrade() -> None:
    """Drop all tables."""
    op.drop_table('chat_messages')
    op.drop_table('personalized_content')
    op.drop_table('chapter_embeddings')
    op.drop_table('chapters')
    op.drop_table('sessions')
    op.drop_table('users')
