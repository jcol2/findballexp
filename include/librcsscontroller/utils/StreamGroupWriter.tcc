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

namespace librcsscontroller
{
    template<typename T>
    StreamGroupWriter StreamGroupWriter::Write(T data)
    {
        for (auto& s : group_)
        {
            if (stream_prefixes_[s])
            {
                stream_prefixes_[s]->Write(s, identifier_);
                stream_prefixes_[s] = nullptr;
            }
            *s << data;
        }        
        return *this;
    }

    template<typename T>
    StreamGroupWriter operator<<(StreamGroupWriter& os, const T& obj)
    {
        return os.Write(obj);
    }

    template<typename T>
    StreamGroupWriter operator<<(StreamGroupWriter&& os, const T& obj)
    {
        StreamGroupWriter copy{os};
        return os << obj;
    }

}