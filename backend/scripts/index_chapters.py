#!/usr/bin/env python3
"""
Index book chapters into Qdrant vector database.

This script:
1. Reads markdown files from book-source/docs/docs/Chapter-*.md
2. Parses H1 (chapter), H2 (module), H3 (section) headings
3. Splits by H3 sections - each section becomes a Qdrant document
4. Generates embeddings using sentence-transformers
5. Stores in Qdrant with chapter/module/section metadata

Usage:
    python backend/scripts/index_chapters.py

Environment variables required:
    QDRANT_URL: Qdrant Cloud URL
    QDRANT_API_KEY: Qdrant API key
"""

import os
import re
import logging
import sys
from pathlib import Path
from typing import Optional

# Add backend to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance

from src.utils.slug_generator import SlugGenerator
from src.config import settings

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


class ChapterIndexer:
    """Indexes book chapters into Qdrant."""

    def __init__(
        self,
        qdrant_url: str,
        qdrant_api_key: str,
        model_name: str = "all-minilm-l6-v2",
        collection_name: str = "book_passages",
        embedding_dimension: int = 384,
    ) -> None:
        """
        Initialize the indexer.

        Args:
            qdrant_url: Qdrant server URL
            qdrant_api_key: Qdrant API key
            model_name: Sentence transformer model to use
            collection_name: Qdrant collection name
            embedding_dimension: Vector dimension (384 for all-minilm-l6-v2)
        """
        self.qdrant_url = qdrant_url
        self.qdrant_api_key = qdrant_api_key
        self.model_name = model_name
        self.collection_name = collection_name
        self.embedding_dimension = embedding_dimension

        self.model: Optional[SentenceTransformer] = None
        self.client: Optional[QdrantClient] = None
        self.point_id = 1

    def initialize(self) -> None:
        """Initialize embedding model and Qdrant connection."""
        logger.info("Initializing embedding model...")
        self.model = SentenceTransformer(self.model_name)
        logger.info(f"Model loaded: {self.model_name}")

        logger.info(f"Connecting to Qdrant at {self.qdrant_url}...")
        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            prefer_grpc=False,
        )
        logger.info("Connected to Qdrant")

    def ensure_collection(self) -> None:
        """Create collection, deleting and recreating if it exists with wrong dimensions."""
        if not self.client:
            raise RuntimeError("Client not initialized")

        try:
            collection_info = self.client.get_collection(self.collection_name)
            # Check if the collection has the correct vector size
            if collection_info.config.params.vectors.size != self.embedding_dimension:
                logger.warning(
                    f"Collection '{self.collection_name}' has wrong dimensions "
                    f"({collection_info.config.params.vectors.size} vs {self.embedding_dimension}). "
                    f"Deleting and recreating..."
                )
                self.client.delete_collection(self.collection_name)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.embedding_dimension,
                        distance=Distance.COSINE,
                    ),
                )
                logger.info(f"Collection recreated with correct dimensions")
            else:
                logger.info(f"Collection '{self.collection_name}' already exists with correct dimensions")
        except Exception as e:
            logger.info(f"Creating collection '{self.collection_name}'...")
            try:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.embedding_dimension,
                        distance=Distance.COSINE,
                    ),
                )
                logger.info(f"Collection created")
            except Exception as create_error:
                logger.error(f"Failed to create collection: {create_error}")
                raise

    def parse_chapter_file(self, filepath: Path) -> list[dict]:
        """
        Parse a chapter markdown file into sections.

        Expects structure:
        # Chapter N: Chapter Name
        ## Module N.M: Module Name
        ### Section Name
        Content...

        Args:
            filepath: Path to markdown file

        Returns:
            List of section dictionaries with chapter/module/section/text
        """
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        sections = []
        current_chapter = "Unknown Chapter"
        current_module = "Unknown Module"

        # Extract chapter name (H1)
        chapter_match = re.search(r'^#\s+(.+?)$', content, re.MULTILINE)
        if chapter_match:
            current_chapter = chapter_match.group(1).strip()

        # Split by H3 sections
        # Match lines starting with ###
        h3_pattern = r'^###\s+(.+?)$'
        h3_matches = list(re.finditer(h3_pattern, content, re.MULTILINE))

        for i, h3_match in enumerate(h3_matches):
            section_title = h3_match.group(1).strip()
            section_start = h3_match.end()

            # Find the next H3 or end of file
            if i + 1 < len(h3_matches):
                section_end = h3_matches[i + 1].start()
            else:
                section_end = len(content)

            section_text = content[section_start:section_end].strip()

            # Extract module name from the text before this section
            # Look backwards for the most recent H2
            text_before_section = content[:h3_match.start()]
            h2_matches = list(re.finditer(r'^##\s+(.+?)$', text_before_section, re.MULTILINE))
            if h2_matches:
                current_module = h2_matches[-1].group(1).strip()

            if section_text:  # Only include non-empty sections
                sections.append({
                    'chapter': current_chapter,
                    'module': current_module,
                    'section': section_title,
                    'text': section_text,
                })

        logger.info(f"Parsed {len(sections)} sections from {filepath.name}")
        return sections

    def embed_sections(self, sections: list[dict]) -> list[dict]:
        """
        Generate embeddings for sections.

        Args:
            sections: List of section dictionaries

        Returns:
            Sections with added 'embedding' field
        """
        if not self.model:
            raise RuntimeError("Model not initialized")

        logger.info(f"Generating embeddings for {len(sections)} sections...")

        for section in sections:
            text = section['text']
            # Use section title + text for better embeddings
            full_text = f"{section['section']}\n{text}"

            embedding = self.model.encode(
                full_text,
                convert_to_numpy=True,
                show_progress_bar=False
            ).tolist()

            section['embedding'] = embedding

        logger.info(f"Generated embeddings for {len(sections)} sections")
        return sections

    def upload_to_qdrant(self, sections: list[dict]) -> int:
        """
        Upload sections to Qdrant.

        Args:
            sections: List of section dictionaries with embeddings

        Returns:
            Number of points uploaded
        """
        if not self.client:
            raise RuntimeError("Client not initialized")

        points = []

        for section in sections:
            section_anchor = SlugGenerator.slugify(section['section'])

            point = PointStruct(
                id=self.point_id,
                vector=section['embedding'],
                payload={
                    'chapter': section['chapter'],
                    'module': section['module'],
                    'section': section_anchor,
                    'text': section['text'],
                    'tokens': len(section['text'].split()),  # Rough token count
                }
            )
            points.append(point)
            self.point_id += 1

        # Upload to Qdrant
        logger.info(f"Uploading {len(points)} points to Qdrant...")
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        logger.info(f"Successfully uploaded {len(points)} points")
        return len(points)

    def index_chapters_from_dir(self, docs_dir: Path) -> int:
        """
        Index all chapter files from a directory.

        Args:
            docs_dir: Path to directory containing numbered markdown files (01_*.md, 02_*.md, etc.)

        Returns:
            Total number of points indexed
        """
        # Find all numbered markdown files (01_intro.md, 02_ros2.md, etc.)
        chapter_files = sorted(docs_dir.glob('[0-9][0-9]_*.md'))

        if not chapter_files:
            logger.warning(f"No Chapter files found in {docs_dir}")
            return 0

        logger.info(f"Found {len(chapter_files)} chapter files")

        total_points = 0

        for chapter_file in chapter_files:
            logger.info(f"\nProcessing {chapter_file.name}...")

            try:
                # Parse chapter
                sections = self.parse_chapter_file(chapter_file)

                if not sections:
                    logger.warning(f"No sections found in {chapter_file.name}")
                    continue

                # Embed sections
                sections = self.embed_sections(sections)

                # Upload to Qdrant
                num_points = self.upload_to_qdrant(sections)
                total_points += num_points

            except Exception as e:
                logger.error(f"Error processing {chapter_file.name}: {e}")
                continue

        return total_points

    def print_stats(self) -> None:
        """Print collection statistics."""
        if not self.client:
            raise RuntimeError("Client not initialized")

        try:
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"\n=== Collection Statistics ===")
            logger.info(f"Collection: {self.collection_name}")
            logger.info(f"Points: {collection_info.points_count}")
        except Exception as e:
            logger.error(f"Error getting collection stats: {e}")


def main() -> None:
    """Main entry point."""
    # Get paths
    backend_dir = Path(__file__).parent.parent
    docs_dir = backend_dir.parent / "book-source" / "docs"

    logger.info(f"Backend directory: {backend_dir}")
    logger.info(f"Docs directory: {docs_dir}")

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        sys.exit(1)

    # Initialize indexer
    indexer = ChapterIndexer(
        qdrant_url=settings.qdrant_url,
        qdrant_api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name,
    )

    try:
        # Initialize
        indexer.initialize()
        indexer.ensure_collection()

        # Index chapters
        total_points = indexer.index_chapters_from_dir(docs_dir)

        logger.info(f"\n=== Indexing Complete ===")
        logger.info(f"Total points indexed: {total_points}")

        # Print stats
        indexer.print_stats()

    except Exception as e:
        logger.error(f"Indexing failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
