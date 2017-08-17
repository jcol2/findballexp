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
    bool MessageParser::ReadType(T* out)
    {
        ss_ >> *out;
        return ss_.good();
    }

    template<typename T>
    bool MessageParser::ReadVal(T* out)
    {
        if (!ReadType(out))
        {
            return false;
        }

        if (!ReadClose())
        {
            return false;
        }
        return true;
    }

    template<typename T>
    bool MessageParser::ReadKeyVal(const std::string& expected_key, T* out)
    {
        if (!ReadKey(expected_key))
        {
            return false;
        }
        return ReadVal(out);
    }

}