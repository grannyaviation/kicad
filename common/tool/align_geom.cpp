/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <tool/align_geom.h>

namespace ALIGN_GEOM
{

std::optional<size_t> SelectTargetIndex( const std::vector<BOX2I>& aBoxes,
                                         const VECTOR2I&           aCursor )
{
    if( aBoxes.empty() )
        return std::nullopt;

    for( size_t i = 0; i < aBoxes.size(); ++i )
    {
        if( aBoxes[i].Contains( aCursor ) )
            return i;
    }

    return 0;
}


std::vector<VECTOR2I> Deltas( const std::vector<BOX2I>& aBoxes, MODE aMode, const BOX2I& aTarget )
{
    std::vector<VECTOR2I> deltas;

    deltas.reserve( aBoxes.size() );

    for( const BOX2I& box : aBoxes )
    {
        switch( aMode )
        {
        case MODE::TOP:
            deltas.emplace_back( 0, aTarget.GetTop() - box.GetTop() );
            break;

        case MODE::BOTTOM:
            deltas.emplace_back( 0, aTarget.GetBottom() - box.GetBottom() );
            break;

        case MODE::LEFT:
            deltas.emplace_back( aTarget.GetLeft() - box.GetLeft(), 0 );
            break;

        case MODE::RIGHT:
            deltas.emplace_back( aTarget.GetRight() - box.GetRight(), 0 );
            break;

        case MODE::CENTER_X:
            deltas.emplace_back( aTarget.Centre().x - box.Centre().x, 0 );
            break;

        case MODE::CENTER_Y:
            deltas.emplace_back( 0, aTarget.Centre().y - box.Centre().y );
            break;
        }
    }

    return deltas;
}

} // namespace ALIGN_GEOM
