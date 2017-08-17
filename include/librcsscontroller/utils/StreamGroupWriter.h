/*
 *  librcsscontroller
 *  A library for controlling rcssserver3d simulations.
 *  Copyright (C) 2017 Jeremy Collette.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBRCSSCONTROLLER_STEAMGROUPWRITER_H_
#define LIBRCSSCONTROLLER_STEAMGROUPWRITER_H_

#include "StreamWriter.h"

#include <ostream>
#include <vector>
#include <unordered_map>

namespace librcsscontroller
{
    /**
     *  The StreamGroupWriter class provides an interface for writing to
     *  multiple streams simultaneously (with the one function call). 
     */
    class StreamGroupWriter
    {
    public:
        /**
         *  Constructor
         *
         *  @param group The group of output streams to write to at the same
         *  time.
         *  @param identifier Some identification of the group, or usage of the
         *  group.
         *  @param prefix The StreamWriters used to write prefixes to specific
         *  streams.
         */
        StreamGroupWriter(std::vector<std::ostream*> group, 
                const std::string& identifier, 
                const std::unordered_map<std::ostream*, StreamWriter*>& prefixes);

        /**
         *  Constructor
         * 
         *  @param group The group of output streams to write to at the same
         *  time.
         */
        StreamGroupWriter(std::vector<std::ostream*> group);

        /**
         *  Writes data to the group of streams.
         *
         *  @tparam T The type of data to be written to the stream.
         *  @param data The data to be written to the stream.
         */
        template<typename T> StreamGroupWriter Write(T data);

    private:
        /**< The group of streams to write to */
        std::vector<std::ostream*> group_;

        /**< The identifier of the group of streams, or their usage */
        std::string identifier_;
        
        /**< The StreamWriter to write prefixes for each stream */
        std::unordered_map<std::ostream*, StreamWriter*> stream_prefixes_;

    };

    /**
     *  Stream Insertion Operator
     *
     *  Writes data to a group of streams.
     * 
     *  @tparam T The type of data to be written.
     *  @param os& The group of streams to write to.
     *  @param obj& The data to write to the streams.
     */
    template<typename T>
    StreamGroupWriter operator<<(StreamGroupWriter& os, const T& obj);

    /**
     *  Stream Insertion Operator
     *
     *  Writes data to a group of streams.
     * 
     *  @tparam T The type of data to be written.
     *  @param os&& The group of streams to write to.
     *  @param obj& The data to write to the streams.
     */
    template<typename T>
    StreamGroupWriter operator<<(StreamGroupWriter&& os, const T& obj);

}

#include "StreamGroupWriter.tcc"

#endif // LIBRCSSCONTROLLER_STEAMGROUPWRITER_H_