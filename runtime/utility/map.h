// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_UTILITY_MAP_H_
#define RUNTIME_UTILITY_MAP_H_

#include <utility>
#include <cstddef>
#include <vector>
#include <map>
#include <unordered_map>
#include <cassert>
namespace diagrammar {

// Utility container
// The class allows random access by index,
// and the address of every contained value is stable.
// Specialize std::allocator<std::pair<Key, T>> and
// std::allocator<std::pair<Key, size_t>> for custom use
template <class Key, class T,
          template <class _Key, class _Value, class... _OtherArgs>
          class MapType = std::unordered_map>
class IndexedMap {
 public:
  // Using only default constructors.
  typedef size_t size_type;
  typedef Key key_type;
  typedef T mapped_type;
  typedef std::pair<Key, T> value_type;
  // iterator is NOT invalidated when inserting new element, but the user has to
  // call the * and -> operator again to get the correct value;
  class iterator {
   public:
    typedef iterator self_type;
    typedef IndexedMap::value_type value_type;
    typedef IndexedMap::value_type& reference;
    typedef IndexedMap::value_type* pointer;

    iterator() = default;
    iterator(const self_type&) = default;
    self_type& operator=(const self_type&) = default;
    iterator(self_type&&) = default;
    self_type& operator=(self_type&&) = default;
    iterator(typename MapType<Key, IndexedMap::size_type>::iterator map_itr,
             std::vector<value_type>* container_ptr)
        : itr_(map_itr), container_ptr_(container_ptr) {}
    // Overloaded operators.
    self_type& operator++() {
      ++itr_;
      return *this;
    }
    reference operator*() const { return (*container_ptr_)[itr_->second]; }
    pointer operator->() const { return container_ptr_->data() + itr_->second; }
    bool operator==(const self_type& rhs) const {
      return itr_ == rhs.itr_ && container_ptr_ == rhs.container_ptr_;
    }
    bool operator!=(const self_type& rhs) const {
      return itr_ != rhs.itr_ || container_ptr_ != rhs.container_ptr_;
    }

   private:
    typename MapType<Key, size_t>::iterator itr_;
    std::vector<value_type>* container_ptr_ = nullptr;
  };

  class const_iterator {
   public:
    typedef const_iterator self_type;
    typedef IndexedMap::value_type value_type;
    typedef const IndexedMap::value_type& reference;
    typedef const IndexedMap::value_type* pointer;

    const_iterator() = default;
    const_iterator(const self_type&) = default;
    self_type& operator=(const self_type&) = default;
    const_iterator(self_type&&) = default;
    self_type& operator=(self_type&&) = default;
    const_iterator(
        typename MapType<Key, IndexedMap::size_type>::const_iterator map_itr,
        const std::vector<value_type>* container_ptr)
        : itr_(map_itr), container_ptr_(container_ptr) {}
    // Overloaded operators.
    self_type& operator++() {
      ++itr_;
      return *this;
    }
    reference operator*() const { return (*container_ptr_)[itr_->second]; }
    pointer operator->() const { return container_ptr_->data() + itr_->second; }
    bool operator==(const self_type& rhs) const {
      return itr_ == rhs.itr_ && container_ptr_ == rhs.container_ptr_;
    }
    bool operator!=(const self_type& rhs) const {
      return itr_ != rhs.itr_ || container_ptr_ != rhs.container_ptr_;
    }

   private:
    typename MapType<Key, size_t>::const_iterator itr_;
    const std::vector<value_type>* container_ptr_ = nullptr;
  };

  iterator begin() { return iterator(lookup_table_.begin(), &container_); }
  iterator end() { return iterator(lookup_table_.end(), &container_); }
  const_iterator begin() const {
    return const_iterator(lookup_table_.begin(), &container_);
  }
  const_iterator end() const {
    return const_iterator(lookup_table_.end(), &container_);
  }

  const_iterator cbegin() const {
    return const_iterator(lookup_table_.begin(), &container_);
  }
  const_iterator cend() const {
    return const_iterator(lookup_table_.end(), &container_);
  }

  // iterface
  size_type size() const;
  iterator find(const key_type& key);
  const_iterator find(const key_type& key) const;
  // T type must have a default constructor
  mapped_type& operator[](const key_type& key);
  // Access by index (random access)
  const mapped_type& get(size_type index) const;
  // Return count (0 or 1)
  size_type erase(const key_type& key);
  void clear();

 private:
  std::vector<value_type> container_;
  MapType<key_type, size_t> lookup_table_;
};

template <class Key, class T, template <class _Key, class _Value,
                                        class... _OtherArgs> class MapType>
size_t IndexedMap<Key, T, MapType>::size() const {
  return container_.size();
}

template <class Key, class T, template <class _Key, class _Value,
                                        class... _OtherArgs> class MapType>
typename IndexedMap<Key, T, MapType>::iterator
IndexedMap<Key, T, MapType>::find(const key_type& key) {
  return iterator(lookup_table_.find(key), &container_);
}

template <class Key, class T, template <class _Key, class _Value,
                                        class... _OtherArgs> class MapType>
typename IndexedMap<Key, T, MapType>::const_iterator
IndexedMap<Key, T, MapType>::find(const key_type& key) const {
  return const_iterator(lookup_table_.find(key), &container_);
}

template <class Key, class T, template <class _Key, class _Value,
                                        class... _OtherArgs> class MapType>
const typename IndexedMap<Key, T, MapType>::mapped_type&
IndexedMap<Key, T, MapType>::get(size_type index) const {
  return container_[index].second;
}

template <class Key, class T, template <class _Key, class _Value,
                                        class... _OtherArgs> class MapType>
typename IndexedMap<Key, T, MapType>::mapped_type& IndexedMap<Key, T, MapType>::
operator[](const key_type& key) {
  auto itr = lookup_table_.find(key);
  if (itr != lookup_table_.end()) return container_[itr->second].second;
  // Create a new element with the key;
  container_.emplace_back(std::make_pair(key, T()));
  lookup_table_[key] = container_.size() - 1;
  return container_.back().second;
}

template <class Key, class T, template <class _Key, class _Value,
                                        class... _OtherArgs> class MapType>
typename IndexedMap<Key, T, MapType>::size_type
IndexedMap<Key, T, MapType>::erase(const key_type& key) {
  auto itr = lookup_table_.find(key);
  if (itr != lookup_table_.end()) {
    size_t idx = itr->second;
    size_t last_idx = container_.size() - 1;
    if (idx != last_idx) {
      // Swap with last element
      std::swap(container_[idx], container_[last_idx]);
      lookup_table_[container_[idx].first] = idx;
    }
    // Erase the last element
    container_.pop_back();
    lookup_table_.erase(itr);
    return 1;
  }
  return 0;
}

template <class Key, class T, template <class _Key, class _Value,
                                        class... _OtherArgs> class MapType>
void IndexedMap<Key, T, MapType>::clear() {
  container_.clear();
  lookup_table_.clear();
}
}  // namespace diagrammar

#endif  // RUNTIME_UTILITY_MAP_H_
