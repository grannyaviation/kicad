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
#include <cmath>
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


std::vector<ALIGNMENT_GUIDE_ENGINE::CLUSTER>
ALIGNMENT_GUIDE_ENGINE::buildClusters( const BOX2I& aMoving, int aAxis ) const
{
    const SPAN           crossMs = spanOf( aMoving, 1 - aAxis );
    std::vector<CLUSTER> clusters;

    for( const BOX2I& n : m_neighbors )
    {
        const SPAN cs = spanOf( n, 1 - aAxis );

        if( !spansOverlap( cs, crossMs ) )
            continue;

        const SPAN s = spanOf( n, aAxis );
        clusters.push_back( { s.Min, s.Max, cs.Min, cs.Max } );
    }

    std::sort( clusters.begin(), clusters.end(),
               []( const CLUSTER& a, const CLUSTER& b ) { return a.Min < b.Min; } );

    if( clusters.empty() )
        return clusters;

    // Standard in-place interval merge.  `>` rather than `>=` so boxes that merely
    // touch still merge, which is what keeps every surviving gap strictly positive.
    size_t w = 0;

    for( size_t r = 1; r < clusters.size(); ++r )
    {
        if( clusters[r].Min > clusters[w].Max )
        {
            clusters[++w] = clusters[r];
        }
        else
        {
            clusters[w].Max = std::max( clusters[w].Max, clusters[r].Max );
            clusters[w].CrossMin = std::min( clusters[w].CrossMin, clusters[r].CrossMin );
            clusters[w].CrossMax = std::max( clusters[w].CrossMax, clusters[r].CrossMax );
        }
    }

    clusters.resize( w + 1 );
    return clusters;
}


void ALIGNMENT_GUIDE_ENGINE::collectAxisCandidates( const BOX2I& aMoving, int aAxis,
                                                    const std::vector<CLUSTER>& aClusters,
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

    // Equal-spacing: for each pair of adjacent clusters, offer positions that extend
    // the pair's gap on either side.  Clusters, not raw neighbors: the gap between two
    // clusters is empty by construction, whereas the gap between two boxes picked by
    // sort order can run through a third box.
    for( size_t k = 0; k + 1 < aClusters.size(); ++k )
    {
        const CLUSTER& left = aClusters[k];
        const CLUSTER& right = aClusters[k + 1];
        const int      gap = right.Min - left.Max; // > 0: touching clusters were merged

        // Moving box after the right cluster with the same gap
        aOut.push_back( { ( right.Max + gap ) - ms.Min, KIND_EQUAL_GAP, k, k + 1, 0 } );

        // Moving box before the left cluster with the same gap.  Anchored on the
        // cluster edge, not on whichever box happened to sort first.
        aOut.push_back( { ( left.Min - gap ) - ms.Max, KIND_EQUAL_GAP, k + 1, k, 0 } );

        // Moving box centered between the pair, if it fits.  Odd leftover room
        // truncates, so the two resulting gaps can differ by one unit.
        if( gap >= ms.Size() )
        {
            const int targetMin = left.Max + ( gap - ms.Size() ) / 2;
            aOut.push_back( { targetMin - ms.Min, KIND_BETWEEN, k, k + 1, 0 } );
        }
    }

    // Center inside a container (board outline, enclosing bbox)
    for( size_t i = 0; i < m_containers.size(); ++i )
    {
        const SPAN cs = spanOf( m_containers[i], aAxis );
        aOut.push_back( { cs.Center() - ms.Center(), KIND_CONTAINER, i, i, 0 } );
    }
}


void ALIGNMENT_GUIDE_ENGINE::buildGraphics( const BOX2I& aSnapped, int aAxis,
                                            const SNAP_CANDIDATE&       aWinner,
                                            const std::vector<CLUSTER>& aClusters,
                                            RESULT&                     aResult ) const
{
    // See the header for what N1/N2 index in each case: the convention differs per kind.
    switch( aWinner.Kind )
    {
    case KIND_ALIGN:
    {
        // Guide line runs along the snapped ordinate, spanning both boxes on the
        // cross axis.  The ordinate was recorded when the candidate was collected.
        const int  ord = aWinner.Ord;
        const SPAN crossM = spanOf( aSnapped, 1 - aAxis );
        const SPAN crossN = spanOf( m_neighbors[aWinner.N1], 1 - aAxis );
        const int  lo = std::min( crossM.Min, crossN.Min );
        const int  hi = std::max( crossM.Max, crossN.Max );

        if( aAxis == 0 )
            aResult.Lines.emplace_back( VECTOR2I( ord, lo ), VECTOR2I( ord, hi ) );
        else
            aResult.Lines.emplace_back( VECTOR2I( lo, ord ), VECTOR2I( hi, ord ) );

        break;
    }

    case KIND_EQUAL_GAP:
    {
        // Not named far/near: those are legacy Windows macros.
        const CLUSTER& cFar = aClusters[aWinner.N1];
        const CLUSTER& cNear = aClusters[aWinner.N2];
        const SPAN     sMov = spanOf( aSnapped, aAxis );

        const SPAN crossMov = spanOf( aSnapped, 1 - aAxis );
        const int  crossMid = ( std::max( cNear.CrossMin, crossMov.Min )
                                + std::min( cNear.CrossMax, crossMov.Max ) ) / 2;

        // Sound only because the gap is strictly positive: "after" puts the moving
        // box at cNear.Max + gap, "before" puts its *far* edge at cNear.Min - gap, so
        // the two cases cannot both satisfy this.  A zero gap would collapse them
        // and send the "after" case down the "before" branch, emitting negative
        // badges — buildClusters merges touching neighbors to prevent exactly that.
        if( sMov.Min > cNear.Max ) // moving sits after the pair
        {
            pushBadge( aResult, aAxis, crossMid, cFar.Max, cNear.Min );
            pushBadge( aResult, aAxis, crossMid, cNear.Max, sMov.Min );
        }
        else // moving sits before the pair
        {
            pushBadge( aResult, aAxis, crossMid, sMov.Max, cNear.Min );
            pushBadge( aResult, aAxis, crossMid, cNear.Max, cFar.Min );
        }

        break;
    }

    case KIND_BETWEEN:
    {
        const CLUSTER& left = aClusters[aWinner.N1];
        const CLUSTER& right = aClusters[aWinner.N2];
        const SPAN     sMov = spanOf( aSnapped, aAxis );

        const SPAN crossMov = spanOf( aSnapped, 1 - aAxis );
        const int  crossMid = crossMov.Min + crossMov.Size() / 2;

        pushBadge( aResult, aAxis, crossMid, left.Max, sMov.Min );
        pushBadge( aResult, aAxis, crossMid, sMov.Max, right.Min );
        break;
    }

    case KIND_CONTAINER:
    {
        const BOX2I&   box = m_containers[aWinner.N1];
        const VECTOR2I center( spanOf( box, 0 ).Center(), spanOf( box, 1 ).Center() );

        // One mark per snap, even if both axes won on the same container.  At most
        // two marks are ever pushed (one per axis), so comparing back() is a full
        // duplicate check, not just a neighbouring-element one.
        if( aResult.CenterMarks.empty() || aResult.CenterMarks.back() != center )
            aResult.CenterMarks.push_back( center );

        break;
    }

    default:
        break;
    }
}


std::optional<ALIGNMENT_GUIDE_ENGINE::RESULT>
ALIGNMENT_GUIDE_ENGINE::FindSnap( const BOX2I& aMoving, int aSnapRange,
                                  const std::optional<VECTOR2D>& aGridStep ) const
{
    RESULT result;
    result.Offset = VECTOR2I( 0, 0 );

    std::optional<SNAP_CANDIDATE> winners[2];
    std::vector<CLUSTER>          clusters[2];

    for( int axis = 0; axis < 2; ++axis )
    {
        clusters[axis] = buildClusters( aMoving, axis );

        std::vector<SNAP_CANDIDATE> candidates;
        // 5 alignment candidates per neighbor, plus up to 3 (2 equal-gap + 1 between)
        // per adjacent cluster pair, of which there are fewer than m_neighbors.size().
        candidates.reserve( 8 * m_neighbors.size() + m_containers.size() );
        collectAxisCandidates( aMoving, axis, clusters[axis], candidates );

        std::optional<SNAP_CANDIDATE> best;

        for( const SNAP_CANDIDATE& c : candidates )
        {
            if( aGridStep )
            {
                const double g = ( axis == 0 ) ? aGridStep->x : aGridStep->y;

                if( g > 0 )
                {
                    const double steps = c.Delta / g;

                    // Reject, never round: a rounded offset would leave the item off the
                    // alignment the guide line is about to claim.
                    if( std::abs( steps - std::round( steps ) ) > 1e-6 )
                        continue;
                }
            }

            if( std::abs( c.Delta ) > aSnapRange )
                continue;

            // Smallest |Delta| in range wins.  Strict < keeps the first-pushed
            // candidate on a tie, and collectAxisCandidates pushes in a fixed order,
            // so the full selection rule is:
            //
            //  1. alignment beats every other kind, because the whole alignment loop
            //     runs before any gap or container candidate is pushed;
            //  2. within alignment, neighbors in input order, and within one neighbor
            //     the pairing order min-min, max-min, min-max, max-max, center-center
            //     — so an edge beats a center on a tie, and Ord is that edge's;
            //  3. then gap kinds, cluster pairs left to right, each pair pushing
            //     after -> before -> between;
            //  4. containers last.
            //
            // The order is load-bearing, not incidental: rule 2 decides which ordinate
            // a multi-way edge tie draws its guide on (see ContainerLosesToNearerAlignment),
            // and rule 1 means a tie between an alignment and a gap or container
            // candidate always renders as a guide line rather than as badges or a
            // center mark.  Reordering the pushes in collectAxisCandidates silently
            // changes both.
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
            buildGraphics( snapped, axis, *winners[axis], clusters[axis], result );
    }

    return result;
}
