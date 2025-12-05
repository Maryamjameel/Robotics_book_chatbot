"""Unit tests for markdown parser service."""

import pytest

from src.services import parse_all_chapters, parse_chapter


class TestParseChapter:
    """Tests for parse_chapter function."""

    def test_parse_chapter_success(self, temp_chapters_dir):
        """Test successful parsing of a single chapter."""
        import os

        ch01_path = os.path.join(temp_chapters_dir, "ch01.md")
        chunks = parse_chapter(ch01_path, "ch01")

        assert len(chunks) > 0
        assert chunks[0].chapter_id == "ch01"
        assert chunks[0].chapter_title == "Introduction to Robotics"
        assert all(chunk.section_number > 0 for chunk in chunks)
        assert all(chunk.content for chunk in chunks)

    def test_parse_chapter_file_not_found(self):
        """Test error when chapter file does not exist."""
        with pytest.raises(FileNotFoundError):
            parse_chapter("/nonexistent/path/file.md", "ch01")

    def test_parse_chapter_metadata(self, temp_chapters_dir):
        """Test that metadata is correctly populated."""
        import os

        ch01_path = os.path.join(temp_chapters_dir, "ch01.md")
        chunks = parse_chapter(ch01_path, "ch01")

        for chunk in chunks:
            assert "chapter_id" in chunk.metadata
            assert "source_file" in chunk.metadata
            assert "section_number" in chunk.metadata
            assert "section_title" in chunk.metadata

    def test_parse_chapter_section_numbering(self, temp_chapters_dir):
        """Test that sections are numbered sequentially."""
        import os

        ch01_path = os.path.join(temp_chapters_dir, "ch01.md")
        chunks = parse_chapter(ch01_path, "ch01")

        section_numbers = [chunk.section_number for chunk in chunks]
        assert section_numbers == list(range(1, len(chunks) + 1))

    @pytest.mark.unit
    def test_parse_chapter_empty_file(self, tmp_path):
        """Test error when chapter file is empty."""
        empty_file = tmp_path / "empty.md"
        empty_file.write_text("")

        with pytest.raises(ValueError, match="empty"):
            parse_chapter(str(empty_file), "empty")

    @pytest.mark.unit
    def test_parse_chapter_no_sections(self, tmp_path):
        """Test error when chapter has no ## sections."""
        no_sections = tmp_path / "nosections.md"
        no_sections.write_text("# Chapter\nSome content without sections")

        with pytest.raises(ValueError, match="No sections"):
            parse_chapter(str(no_sections), "nosec")


class TestParseAllChapters:
    """Tests for parse_all_chapters function."""

    def test_parse_all_chapters_success(self, temp_chapters_dir):
        """Test successful parsing of all chapters in directory."""
        chunks = parse_all_chapters(temp_chapters_dir)

        assert len(chunks) > 0
        assert all(chunk.chapter_id for chunk in chunks)
        assert all(chunk.content for chunk in chunks)

    def test_parse_all_chapters_directory_not_found(self):
        """Test error when directory does not exist."""
        with pytest.raises(FileNotFoundError):
            parse_all_chapters("/nonexistent/directory")

    def test_parse_all_chapters_multiple_files(self, temp_chapters_dir):
        """Test parsing multiple chapter files."""
        chunks = parse_all_chapters(temp_chapters_dir)

        # Should have chapters from ch01, ch02, ch03
        chapter_ids = set(chunk.chapter_id for chunk in chunks)
        assert "ch01" in chapter_ids
        assert "ch02" in chapter_ids
        assert "ch03" in chapter_ids

    def test_parse_all_chapters_empty_directory(self, tmp_path):
        """Test parsing empty directory."""
        from src.services import parse_all_chapters as parse_func

        chunks = parse_func(str(tmp_path))
        assert chunks == []

    @pytest.mark.unit
    def test_parse_all_chapters_sorted_files(self, temp_chapters_dir):
        """Test that files are processed in sorted order."""
        chunks = parse_all_chapters(temp_chapters_dir)

        # Verify chapters are in order
        chapters = []
        for chunk in chunks:
            if not chapters or chapters[-1] != chunk.chapter_id:
                chapters.append(chunk.chapter_id)

        assert chapters == sorted(chapters)
