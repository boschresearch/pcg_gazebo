# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


class _CollectionManager(object):
    """Base class for managing collections."""
    _INSTANCE = None

    def __init__(self):
        self._collection = dict()

    @property
    def tags(self):
        """`list`: List of strings of all tags in the collection"""
        return self._collection.keys()

    @property
    def size(self):
        """`int`: Size of collection"""
        return len(self.tags)

    def add(self, element):
        """Add element to the collection.

        > *Input arguments*

        * `element`: New collection element.

        > *Returns*

        `True`, is successfull. `False` otherwise.
        """
        if not self.has_element(element.name):
            self._collection[element.name] = element
            return True
        else:
            return False

    def create_empty(self, *args, **kwargs):
        raise NotImplementedError()

    def remove(self, tag):
        """Remove an element from the collection.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the element to be removed.

        > *Returns*

        `True`, if element could be removed. `False` if `tag` is invalid.
        """
        if self.has_element(tag):
            del self._collection[tag]
            return True
        return False

    def get(self, tag):
        """Return an element from the collection.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the element

        > *Returns*

        Collection element. `None` if `tag` is invalid.
        """
        if self.has_element(tag):
            return self._collection[tag]
        else:
            return None

    def has_element(self, tag):
        """Return `True` if an element for `tag` exists.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the element.
        """
        return tag in self.tags
