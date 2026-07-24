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

#include <algorithm>
#include <cstdlib>

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

bool spansOverlap( const SPAN& aA, const SPAN& aB )
{
    return aA.Min <= aB.Max && aB.Min <= aA.Max;
}

/// Append a badge measuring the aFrom..aTo gap along aAxis, drawn at aCrossMid on the
/// cross axis.  Callers decide what "the middle" means for their kind of snap.
void pushBadge( ALIGNMENT_GUIDE_ENGINE::RESULT& aResult, int aAxis, int aCrossMid, int aFrom,
                int aTo )
{
    ALIGNMENT_GUIDE_ENGINE::GAP_BADGE badge;
    badge.Gap = aTo - aFrom;
    badge.Vertical = ( aAxis == 1 );

    const int mid = aFrom + badge.Gap / 2;
    badge.Pos = ( aAxis == 0 ) ? VECTOR2I( mid, aCrossMid ) : VECTOR2I( aCrossMid, mid );
    aResult.Badges.push_back( badge );
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

        aOut.push_back( { ns.Min - ms.Min, KIND_ALIGN, i, i, ns.Min } );
        aOut.push_back( { ns.Max - ms.Min, KIND_ALIGN, i, i, ns.Max } );
        aOut.push_back( { ns.Min - ms.Max, KIND_ALIGN, i, i, ns.Min } );
        aOut.push_back( { ns.Max - ms.Max, KIND_ALIGN, i, i, ns.Max } );
        aOut.push_back( { ns.Center() - ms.Center(), KIND_ALIGN, i, i, ns.Center() } );
    }

    // Equal-spacing: for each pair of neighbors adjacent along this axis whose
    // cross-axis spans overlap the moving box, offer positions that extend the
    // pair's gap on either side.
    const SPAN crossMs = spanOf( aMoving, 1 - aAxis );

    std::vector<size_t> overlapping;

    for( size_t i = 0; i < m_neighbors.size(); ++i )
    {
        if( spansOverlap( spanOf( m_neighbors[i], 1 - aAxis ), crossMs ) )
            overlapping.push_back( i );
    }

    std::sort( overlapping.begin(), overlapping.end(),
               [&]( size_t a, size_t b )
               {
                   return spanOf( m_neighbors[a], aAxis ).Min
                          < spanOf( m_neighbors[b], aAxis ).Min;
               } );

    for( size_t k = 0; k + 1 < overlapping.size(); ++k )
    {
        const size_t i = overlapping[k];
        const size_t j = overlapping[k + 1];
        const SPAN   si = spanOf( m_neighbors[i], aAxis );
        const SPAN   sj = spanOf( m_neighbors[j], aAxis );
        const int    gap = sj.Min - si.Max;

        if( gap < 0 )
            continue; // overlapping neighbors: no meaningful gap

        // Moving box after j with the same gap: moving.Min = j.Max + gap
        aOut.push_back( { ( sj.Max + gap ) - ms.Min, KIND_EQUAL_GAP, i, j, 0 } );

        // Moving box before i with the same gap: moving.Max = i.Min - gap
        aOut.push_back( { ( si.Min - gap ) - ms.Max, KIND_EQUAL_GAP, j, i, 0 } );

        // Moving box centered between the pair, if it fits.  Odd leftover room
        // truncates, so the two resulting gaps can differ by one unit.
        if( gap >= ms.Size() )
        {
            const int targetMin = si.Max + ( gap - ms.Size() ) / 2;
            aOut.push_back( { targetMin - ms.Min, KIND_BETWEEN, i, j, 0 } );
        }
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
        // cross axis.  The ordinate was recorded when the candidate was collected.
        const int  ord = aWinner.Ord;
        const SPAN crossM = spanOf( aSnapped, 1 - aAxis );
        const SPAN crossN = spanOf( other, 1 - aAxis );
        const int  lo = std::min( crossM.Min, crossN.Min );
        const int  hi = std::max( crossM.Max, crossN.Max );

        if( aAxis == 0 )
            aResult.Lines.emplace_back( VECTOR2I( ord, lo ), VECTOR2I( ord, hi ) );
        else
            aResult.Lines.emplace_back( VECTOR2I( lo, ord ), VECTOR2I( hi, ord ) );
    }

    if( aWinner.Kind == KIND_EQUAL_GAP )
    {
        // N1 = far neighbor, N2 = near neighbor (the one adjacent to the moving box)
        const SPAN sFar = spanOf( m_neighbors[aWinner.N1], aAxis );
        const SPAN sNear = spanOf( m_neighbors[aWinner.N2], aAxis );
        const SPAN sMov = spanOf( aSnapped, aAxis );

        const SPAN crossNear = spanOf( m_neighbors[aWinner.N2], 1 - aAxis );
        const SPAN crossMov = spanOf( aSnapped, 1 - aAxis );
        const int  crossMid = ( std::max( crossNear.Min, crossMov.Min )
                                + std::min( crossNear.Max, crossMov.Max ) ) / 2;

        if( sMov.Min > sNear.Max ) // moving sits after the pair
        {
            pushBadge( aResult, aAxis, crossMid, sFar.Max, sNear.Min );
            pushBadge( aResult, aAxis, crossMid, sNear.Max, sMov.Min );
        }
        else // moving sits before the pair
        {
            pushBadge( aResult, aAxis, crossMid, sMov.Max, sNear.Min );
            pushBadge( aResult, aAxis, crossMid, sNear.Max, sFar.Min );
        }
    }

    if( aWinner.Kind == KIND_BETWEEN )
    {
        // N1 = neighbor before the moving box, N2 = neighbor after it
        const SPAN sLeft = spanOf( m_neighbors[aWinner.N1], aAxis );
        const SPAN sRight = spanOf( m_neighbors[aWinner.N2], aAxis );
        const SPAN sMov = spanOf( aSnapped, aAxis );

        const SPAN crossMov = spanOf( aSnapped, 1 - aAxis );
        const int  crossMid = crossMov.Min + crossMov.Size() / 2;

        pushBadge( aResult, aAxis, crossMid, sLeft.Max, sMov.Min );
        pushBadge( aResult, aAxis, crossMid, sMov.Max, sRight.Min );
    }
}


std::optional<ALIGNMENT_GUIDE_ENGINE::RESULT>
ALIGNMENT_GUIDE_ENGINE::FindSnap( const BOX2I& aMoving, int aSnapRange ) const
{
    RESULT result;
    result.Offset = VECTOR2I( 0, 0 );

    std::optional<SNAP_CANDIDATE> winners[2];

    for( int axis = 0; axis < 2; ++axis )
    {
        std::vector<SNAP_CANDIDATE> candidates;
        // 5 alignment candidates per neighbor, plus up to 3 (2 equal-gap + 1 between)
        // per adjacent pair, of which there are fewer than m_neighbors.size().
        candidates.reserve( 8 * m_neighbors.size() + m_containers.size() );
        collectAxisCandidates( aMoving, axis, candidates );

        std::optional<SNAP_CANDIDATE> best;

        for( const SNAP_CANDIDATE& c : candidates )
        {
            if( std::abs( c.Delta ) > aSnapRange )
                continue;

            // Strict <: ties keep the first-pushed candidate, so the preference order
            // is min -> max -> center within a neighbor, then by neighbor index.
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
