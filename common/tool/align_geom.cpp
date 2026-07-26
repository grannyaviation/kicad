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
#include <algorithm>

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


std::optional<BOX2I> CellAt( const std::vector<SEG>& aSegments, const VECTOR2I& aPoint )
{
    std::optional<int> left, right, top, bottom;

    for( const SEG& seg : aSegments )
    {
        const bool vertical = seg.A.x == seg.B.x;
        const bool horizontal = seg.A.y == seg.B.y;

        // Equal means either a diagonal (neither) or a degenerate point (both).  Neither bounds
        // anything rectilinear, and taking an endpoint as a wall would put a cell edge at an
        // arbitrary place.
        if( vertical == horizontal )
            continue;

        if( vertical )
        {
            if( aPoint.y < std::min( seg.A.y, seg.B.y ) || aPoint.y > std::max( seg.A.y, seg.B.y ) )
                continue;

            // Strict: a segment through aPoint belongs to neither side.
            if( seg.A.x > aPoint.x && ( !right || seg.A.x < *right ) )
                right = seg.A.x;
            else if( seg.A.x < aPoint.x && ( !left || seg.A.x > *left ) )
                left = seg.A.x;
        }
        else
        {
            if( aPoint.x < std::min( seg.A.x, seg.B.x ) || aPoint.x > std::max( seg.A.x, seg.B.x ) )
                continue;

            if( seg.A.y > aPoint.y && ( !bottom || seg.A.y < *bottom ) )
                bottom = seg.A.y;
            else if( seg.A.y < aPoint.y && ( !top || seg.A.y > *top ) )
                top = seg.A.y;
        }
    }

    if( !left || !right || !top || !bottom )
        return std::nullopt;

    return BOX2I( VECTOR2I( *left, *top ), VECTOR2I( *right - *left, *bottom - *top ) );
}

} // namespace ALIGN_GEOM
