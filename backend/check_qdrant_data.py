"""Check Qdrant collection data structure."""

from qdrant_client import QdrantClient
from src.config import config
import json

def check_collection():
    """Check what data is actually in Qdrant."""
    client = QdrantClient(url=config.qdrant_url, api_key=config.qdrant_api_key)

    # Get collection info
    info = client.get_collection(config.collection_name)
    print(f"Collection: {config.collection_name}")
    print(f"Points count: {info.points_count}")
    print(f"Vector size: {info.config.params.vectors.size}")
    print("")

    # Get a few sample points
    points, _ = client.scroll(
        collection_name=config.collection_name,
        limit=3,
        with_payload=True,
        with_vectors=False
    )

    print(f"Sample of {len(points)} points:")
    print("=" * 60)

    for i, point in enumerate(points):
        print(f"\nPoint {i+1} (ID: {point.id}):")
        print(f"Payload keys: {list(point.payload.keys())}")
        print(f"Payload:")
        print(json.dumps(point.payload, indent=2))

        # Check for missing fields
        expected_fields = ["text", "chapter_id", "chapter_title", "section_number", "section_title"]
        missing = [f for f in expected_fields if f not in point.payload]
        if missing:
            print(f"[WARNING] Missing fields: {missing}")
        print("-" * 60)


if __name__ == "__main__":
    check_collection()
