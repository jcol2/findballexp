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

#ifndef LIBRCSSCONTROLLER_DATETIMEWRITER_H_
#define LIBRCSSCONTROLLER_DATETIMEWRITER_H_

#include "StreamWriter.h"

#include <iomanip>

namespace librcsscontroller
{
    /**
     *  The DateTimeWriter class is a StreamWriter which writes the current date
     *  and time to an out stream.
     */
    class DateTimeWriter
        : public StreamWriter
    {
    public:
        /**
         *  Writes the current date and time, and the name of a LogLevel to
         *  an output stream.
         *
         *  @param s The output stream to write to.
         *  @param log_level The name of the LogLevel being used.
         */
        void virtual Write(std::ostream* s, const std::string& log_level);
    };

}

#endif // LIBRCSSCONTROLLER_DATETIMEPREFIXWRITER_H_