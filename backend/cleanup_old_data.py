"""Cleanup script to remove old Qdrant points missing metadata fields."""

import sys
from qdrant_client import QdrantClient
from src.config import config


def cleanup_old_points():
    """Remove points from Qdrant that are missing required metadata fields."""
    print("=" * 60)
    print("Qdrant Cleanup: Removing Old Data")
    print("=" * 60)

    # Connect to Qdrant
    client = QdrantClient(url=config.qdrant_url, api_key=config.qdrant_api_key)

    # Get collection info
    collection_name = config.collection_name
    info = client.get_collection(collection_name)
    initial_count = info.points_count

    print(f"\nCollection: {collection_name}")
    print(f"Initial point count: {initial_count}")
    print("")

    # Required fields for valid points
    required_fields = ["text", "chapter_title"]

    # Scroll through all points and identify invalid ones
    print("Scanning for invalid points...")
    invalid_point_ids = []
    valid_count = 0
    batch_size = 100

    offset = None
    while True:
        # Scroll through points
        points, next_offset = client.scroll(
            collection_name=collection_name,
            limit=batch_size,
            offset=offset,
            with_payload=True,
            with_vectors=False
        )

        if not points:
            break

        # Check each point for required fields
        for point in points:
            payload_keys = point.payload.keys()
            missing_fields = [f for f in required_fields if f not in payload_keys]

            if missing_fields:
                invalid_point_ids.append(point.id)
                print(f"  [INVALID] Point {point.id}: Missing {missing_fields}")
            else:
                valid_count += 1

        # Update offset for next iteration
        offset = next_offset
        if offset is None:
            break

    print(f"\nScan complete:")
    print(f"  Valid points: {valid_count}")
    print(f"  Invalid points: {len(invalid_point_ids)}")

    # Delete invalid points if any found
    if invalid_point_ids:
        print(f"\nDeleting {len(invalid_point_ids)} invalid points...")

        try:
            client.delete(
                collection_name=collection_name,
                points_selector=invalid_point_ids
            )
            print("[OK] Deletion successful!")

            # Verify deletion
            new_info = client.get_collection(collection_name)
            final_count = new_info.points_count

            print(f"\nVerification:")
            print(f"  Before: {initial_count} points")
            print(f"  After: {final_count} points")
            print(f"  Removed: {initial_count - final_count} points")

            if final_count == valid_count:
                print("\n[SUCCESS] Cleanup completed! All invalid points removed.")
                return 0
            else:
                print(f"\n[WARNING] Point count mismatch. Expected {valid_count}, got {final_count}")
                return 1

        except Exception as e:
            print(f"\n[ERROR] Deletion failed: {str(e)}")
            return 1

    else:
        print("\n[OK] No invalid points found. Collection is clean!")
        return 0


def main():
    """Main entry point."""
    try:
        # Ask for confirmation
        print("\nThis script will remove points missing 'text' and 'chapter_title' fields.")
        print("This action cannot be undone.")
        response = input("\nContinue? (yes/no): ").strip().lower()

        if response not in ["yes", "y"]:
            print("\nCleanup cancelled.")
            sys.exit(0)

        # Run cleanup
        exit_code = cleanup_old_points()
        sys.exit(exit_code)

    except KeyboardInterrupt:
        print("\n\nCleanup cancelled by user.")
        sys.exit(1)

    except Exception as e:
        print(f"\n[ERROR] Cleanup failed: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
