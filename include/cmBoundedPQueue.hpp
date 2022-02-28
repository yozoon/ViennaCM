#ifndef CM_BOUNDED_PQUEUE_HPP
#define CM_BOUNDED_PQUEUE_HPP

/**
 * File: BoundedPQueue.hpp
 * Author: Keith Schwarz (htiek@cs.stanford.edu)
 */

#include <functional>
#include <limits>
#include <map>
#include <utility>

/** For later use with kNN **/
template <class K, class V, typename Comparator = std::less<K>>
struct cmBoundedPQueue {
  using MapType = std::multimap<K, V, Comparator>;
  using SizeType = std::size_t;

private:
  MapType mmap;
  const SizeType maximumSize;

public:
  cmBoundedPQueue(SizeType passedMaximumSize)
      : maximumSize(passedMaximumSize) {}

  void enqueue(std::pair<K, V> &&item) {
    /* Optimization: If this isn't going to be added, don't add it. */
    if (size() == maxSize() && mmap.key_comp()(worst(), item.first))
      return;

    /* Add the element to the collection. */
    mmap.emplace(std::forward<std::pair<K, V>>(item));

    /* If there are too many elements in the queue, drop off the last one. */
    if (size() > maxSize()) {
      auto last = mmap.end();
      --last; // Now points to highest-priority element
      mmap.erase(last);
    }
  }

  V dequeueBest() {
    /* Copy the best value. */
    V result = mmap.begin()->second;

    /* Remove it from the map. */
    mmap.erase(mmap.begin());

    return result;
  }

  SizeType maxSize() const { return maximumSize; }

  SizeType size() const { return mmap.size(); }

  bool empty() const { return mmap.empty(); }

  K best() const {
    return mmap.empty() ? std::numeric_limits<K>::infinity()
                        : mmap.begin()->first;
  }

  K worst() const {
    return mmap.empty() ? std::numeric_limits<K>::infinity()
                        : mmap.rbegin()->first;
  }
};
#endif