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

#include <tool/alignment_guide_engine.h>

#include <cmath>
#include <cstdlib>

#include <math/util.h>

namespace
{
/// Min/max of a box along one axis (axis 0 = X, 1 = Y)
struct SPAN
{
    int Min;
    int Max;

    int Center() const { return Min + ( Max - Min ) / 2; }
    int Size() const { return Max - Min; }
};

SPAN spanOf( const BOX2I& aBox, int aAxis )
{
    if( aAxis == 0 )
        return { aBox.GetLeft(), aBox.GetRight() };

    return { aBox.GetTop(), aBox.GetBottom() };
}

[[maybe_unused]] bool spansOverlap( const SPAN& aA, const SPAN& aB )
{
    return aA.Min <= aB.Max && aB.Min <= aA.Max;
}
} // namespace


void ALIGNMENT_GUIDE_ENGINE::collectAxisCandidates( const BOX2I& aMoving, int aAxis,
                                                    std::vector<SNAP_CANDIDATE>& aOut ) const
{
    const SPAN ms = spanOf( aMoving, aAxis );

    // Edge/center alignment: min-min, min-max, max-min, max-max, center-center.
    // Center-to-edge pairings are deliberately excluded as visual noise.
    for( size_t i = 0; i < m_neighbors.size(); ++i )
    {
        const SPAN ns = spanOf( m_neighbors[i], aAxis );

        aOut.push_back( { ns.Min - ms.Min, KIND_ALIGN, i, i } );
        aOut.push_back( { ns.Max - ms.Min, KIND_ALIGN, i, i } );
        aOut.push_back( { ns.Min - ms.Max, KIND_ALIGN, i, i } );
        aOut.push_back( { ns.Max - ms.Max, KIND_ALIGN, i, i } );
        aOut.push_back( { ns.Center() - ms.Center(), KIND_ALIGN, i, i } );
    }
}


void ALIGNMENT_GUIDE_ENGINE::buildGraphics( const BOX2I& aSnapped, int aAxis,
                                            const SNAP_CANDIDATE& aWinner, RESULT& aResult ) const
{
    const BOX2I& other = ( aWinner.Kind == KIND_CONTAINER ) ? m_containers[aWinner.N1]
                                                            : m_neighbors[aWinner.N1];

    if( aWinner.Kind == KIND_ALIGN )
    {
        // Guide line runs along the snapped ordinate, spanning both boxes on the
        // cross axis.
        const SPAN ms = spanOf( aSnapped, aAxis );
        const SPAN ns = spanOf( other, aAxis );

        // Find which ordinate actually aligned (one of ms.Min/ms.Max/center)
        int ord;

        if( ms.Min == ns.Min || ms.Min == ns.Max )
            ord = ms.Min;
        else if( ms.Max == ns.Min || ms.Max == ns.Max )
            ord = ms.Max;
        else
            ord = ms.Center();

        const SPAN crossM = spanOf( aSnapped, 1 - aAxis );
        const SPAN crossN = spanOf( other, 1 - aAxis );
        const int  lo = std::min( crossM.Min, crossN.Min );
        const int  hi = std::max( crossM.Max, crossN.Max );

        if( aAxis == 0 )
            aResult.Lines.emplace_back( VECTOR2I( ord, lo ), VECTOR2I( ord, hi ) );
        else
            aResult.Lines.emplace_back( VECTOR2I( lo, ord ), VECTOR2I( hi, ord ) );
    }
}


std::optional<ALIGNMENT_GUIDE_ENGINE::RESULT>
ALIGNMENT_GUIDE_ENGINE::FindSnap( const BOX2I& aMoving, int aSnapRange,
                                  const std::optional<VECTOR2D>& aGrid ) const
{
    RESULT result;
    result.Offset = VECTOR2I( 0, 0 );

    std::optional<SNAP_CANDIDATE> winners[2];

    for( int axis = 0; axis < 2; ++axis )
    {
        std::vector<SNAP_CANDIDATE> candidates;
        collectAxisCandidates( aMoving, axis, candidates );

        std::optional<SNAP_CANDIDATE> best;

        for( SNAP_CANDIDATE& c : candidates )
        {
            if( aGrid )
            {
                // Quantize the offset so items that started on-grid stay on-grid
                double g = ( axis == 0 ) ? aGrid->x : aGrid->y;

                if( g > 0 )
                    c.Delta = KiROUND( KiROUND( c.Delta / g ) * g );
            }

            if( std::abs( c.Delta ) > aSnapRange )
                continue;

            if( !best || std::abs( c.Delta ) < std::abs( best->Delta ) )
                best = c;
        }

        if( best )
        {
            if( axis == 0 )
                result.Offset.x = best->Delta;
            else
                result.Offset.y = best->Delta;

            winners[axis] = best;
        }
    }

    if( !winners[0] && !winners[1] )
        return std::nullopt;

    BOX2I snapped = aMoving;
    snapped.Move( result.Offset );

    for( int axis = 0; axis < 2; ++axis )
    {
        if( winners[axis] )
            buildGraphics( snapped, axis, *winners[axis], result );
    }

    return result;
}
