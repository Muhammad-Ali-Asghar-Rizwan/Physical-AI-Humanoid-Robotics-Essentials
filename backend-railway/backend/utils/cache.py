import time
from typing import Any, Optional
from threading import Lock

class SimpleCache:
    """
    A simple in-memory cache with TTL (Time To Live) functionality.
    """
    def __init__(self, default_ttl: int = 300):  # 5 minutes default TTL
        self._cache = {}
        self._default_ttl = default_ttl
        self._lock = Lock()
    
    def get(self, key: str) -> Optional[Any]:
        """
        Get a value from the cache.
        Returns None if the key doesn't exist or has expired.
        """
        with self._lock:
            if key in self._cache:
                value, expiry = self._cache[key]
                if time.time() < expiry:
                    return value
                else:
                    # Remove expired entry
                    del self._cache[key]
        return None
    
    def set(self, key: str, value: Any, ttl: Optional[int] = None) -> None:
        """
        Set a value in the cache with optional TTL.
        If TTL is not provided, uses default TTL.
        """
        ttl = ttl or self._default_ttl
        expiry = time.time() + ttl
        
        with self._lock:
            self._cache[key] = (value, expiry)
    
    def delete(self, key: str) -> bool:
        """
        Delete a key from the cache.
        Returns True if the key existed, False otherwise.
        """
        with self._lock:
            if key in self._cache:
                del self._cache[key]
                return True
        return False
    
    def clear(self) -> None:
        """
        Clear all entries from the cache.
        """
        with self._lock:
            self._cache.clear()

# Global cache instance for query results
query_cache = SimpleCache(default_ttl=600)  # 10 minutes TTL for query results