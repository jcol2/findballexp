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

#ifndef LIBRCSSCONTROLLER_STREAMWRITER_H_
#define LIBRCSSCONTROLLER_STREAMWRITER_H_

#include <ostream>

namespace librcsscontroller
{
    /**
     *  The StreamWriter class is an abstract base class, where implementing
     *  classes will write some kind of data to a stream. 
     *
     *  The deriving class may write some static or generated data to the
     *  stream, such as an identifier, or the current time.
     */    
    class StreamWriter
    {        
    public:
        /**
         *  Writes some kind of data to the output stream.
         *
         *  @param s[in] The output stream to write to.
         *  @param log_level The identifier of the LogLevel being used.
         */
        void virtual Write(std::ostream* s, const std::string& log_level) = 0;
    
    };

}

#endif // LIBRCSSCONTROLLER_STREAMWRITER_H_