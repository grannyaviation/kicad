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

bool spansOverlap( const SPAN& aA, const SPAN& aB )
{
    return aA.Min <= aB.Max && aB.Min <= aA.Max;
}

/// Append a badge measuring the aFrom..aTo gap along aAxis, drawn at aCrossMid on the
/// cross axis.  Callers decide what "the middle" means for their kind of snap.
void pushBadge( ALIGNMENT_GUIDE_ENGINE::RESULT& aResult, int aAxis, int aCrossMid, int aFrom,
                int aTo, bool aApproximate )
{
    ALIGNMENT_GUIDE_ENGINE::GAP_BADGE badge;
    badge.Gap = aTo - aFrom;
    badge.Vertical = ( aAxis == 1 );
    badge.Approximate = aApproximate;

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
                                                    const std::vector<CLUSTER>&  aClusters,
                                                    int                          aGridStep,
                                                    std::vector<SNAP_CANDIDATE>& aOut ) const
{
    const SPAN ms = spanOf( aMoving, aAxis );

    // An exact candidate: Dist == Delta, so it ranks by how far it moves the item, as always.
    auto push = [&]( int aDelta, int aKind, size_t aN1, size_t aN2, int aOrd )
    {
        aOut.push_back( { aDelta, aDelta, aKind, aN1, aN2, aOrd, false } );
    };

    // A grid-legal fallback for a candidate whose exact position the grid cannot reach.  Emitted
    // *after* the exact one so an exact spacing always takes a tie, and only when rounding
    // actually changes something.
    //
    // Dist stays the exact distance.  Ranking on the rounded Delta would let a candidate 0.25 of
    // a step away round to 0 and then beat every real snap with an unbeatable distance of zero,
    // pinning the item wherever it happened to be.
    auto pushRounded = [&]( int aDelta, int aKind, size_t aN1, size_t aN2 )
    {
        if( aGridStep <= 0 )
            return;

        const int rounded = KiROUND( double( aDelta ) / aGridStep ) * aGridStep;

        // A snap that moves nothing is not a snap.  Anything less than half a grid step from
        // where it already sits rounds to zero, and pushing that produced a candidate ranked by
        // its tiny pre-rounding distance -- so it usually won, the item stayed exactly where it
        // was, and the engine still reported a successful alignment and drew badges for it.
        // Those badges then appeared and persisted under a merely hovering cursor.
        //
        // Rejecting it is the same call the exact path already makes when the grid refuses an
        // offset: silence beats a guide that claims a relationship the item never moved into.
        if( rounded != 0 && rounded != aDelta )
            aOut.push_back( { rounded, aDelta, aKind, aN1, aN2, 0, true } );
    };

    // Edge/center alignment: min-min, min-max, max-min, max-max, center-center.
    // Center-to-edge pairings are deliberately excluded as visual noise.
    for( size_t i = 0; i < m_neighbors.size(); ++i )
    {
        const SPAN ns = spanOf( m_neighbors[i], aAxis );

        push( ns.Min - ms.Min, KIND_ALIGN, i, i, ns.Min );
        push( ns.Max - ms.Min, KIND_ALIGN, i, i, ns.Max );
        push( ns.Min - ms.Max, KIND_ALIGN, i, i, ns.Min );
        push( ns.Max - ms.Max, KIND_ALIGN, i, i, ns.Max );
        push( ns.Center() - ms.Center(), KIND_ALIGN, i, i, ns.Center() );
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
        const int after = ( right.Max + gap ) - ms.Min;

        // Moving box before the left cluster with the same gap.  Anchored on the
        // cluster edge, not on whichever box happened to sort first.
        const int before = ( left.Min - gap ) - ms.Max;

        push( after, KIND_EQUAL_GAP, k, k + 1, 0 );
        push( before, KIND_EQUAL_GAP, k + 1, k, 0 );

        // Rounding shifts the box by up to half a step either way, so a gap with less room than
        // that would be jumped clean over, leaving the box on the wrong side of its own gap and
        // a badge reporting a negative distance.  No fallback for those.
        if( 2 * gap > aGridStep )
        {
            pushRounded( after, KIND_EQUAL_GAP, k, k + 1 );
            pushRounded( before, KIND_EQUAL_GAP, k + 1, k );
        }

        // Moving box centered between the pair, if it fits.  Odd leftover room
        // truncates, so the two resulting gaps can differ by one unit.
        if( gap >= ms.Size() )
        {
            const int targetMin = left.Max + ( gap - ms.Size() ) / 2;

            push( targetMin - ms.Min, KIND_BETWEEN, k, k + 1, 0 );

            // A full step of slack, for the reason the pair above needs half of one.
            if( gap >= ms.Size() + std::max( 0, aGridStep ) )
                pushRounded( targetMin - ms.Min, KIND_BETWEEN, k, k + 1 );
        }
    }

    // Center inside a container (board outline, enclosing bbox)
    for( size_t i = 0; i < m_containers.size(); ++i )
    {
        const SPAN cs = spanOf( m_containers[i], aAxis );

        push( cs.Center() - ms.Center(), KIND_CONTAINER, i, i, 0 );
        pushRounded( cs.Center() - ms.Center(), KIND_CONTAINER, i, i );
    }
}


void ALIGNMENT_GUIDE_ENGINE::buildGapBadges( const BOX2I& aSnapped, int aAxis,
                                             const std::vector<CLUSTER>& aClusters, int aRefGap,
                                             int aTolerance, RESULT& aResult ) const
{
    const SPAN ms = spanOf( aSnapped, aAxis );
    const SPAN crossM = spanOf( aSnapped, 1 - aAxis );

    // The moving box takes part in the run like any other box, so it joins the cluster list and
    // the whole thing is walked in spatial order.
    std::vector<CLUSTER> run = aClusters;
    run.push_back( { ms.Min, ms.Max, crossM.Min, crossM.Max } );

    std::sort( run.begin(), run.end(),
               []( const CLUSTER& a, const CLUSTER& b ) { return a.Min < b.Min; } );

    for( size_t i = 0; i + 1 < run.size(); ++i )
    {
        const int gap = run[i + 1].Min - run[i].Max;

        // The rounding tolerance applies only to a gap the moving box is an end of.  That is the
        // only gap the snap could have changed; a pair of static neighbours has whatever gap it
        // always had, so there is nothing to forgive and it has to match exactly.
        //
        // Applying the tolerance to the whole run badged items the user never touched: on a
        // 100 mil grid any two symbols within 2.54 mm of the reference spacing acquired a badge,
        // which is how badges came to appear in parts of a schematic unrelated to the drag.
        const bool moved = ( run[i].Min == ms.Min && run[i].Max == ms.Max )
                           || ( run[i + 1].Min == ms.Min && run[i + 1].Max == ms.Max );

        // Every gap that matches gets a badge, not only the two the snap was computed from:
        // with four boxes in a column, the equality the user asked for is a property of all
        // three gaps, and showing one of them proves nothing.
        //
        // A non-positive gap means the moving box overlaps that cluster, which is not a gap.
        if( gap <= 0 || std::abs( gap - aRefGap ) > ( moved ? aTolerance : 0 ) )
            continue;

        const int crossMid = ( std::max( run[i].CrossMin, run[i + 1].CrossMin )
                               + std::min( run[i].CrossMax, run[i + 1].CrossMax ) ) / 2;

        // Flagged per gap rather than per snap: within one rounded run some gaps can still come
        // out exactly equal, and those are not approximations.
        pushBadge( aResult, aAxis, crossMid, run[i].Max, run[i + 1].Min, gap != aRefGap );
    }
}


void ALIGNMENT_GUIDE_ENGINE::buildAlignmentLines( const BOX2I& aSnapped, int aAxis, int aWinnerOrd,
                                                  RESULT& aResult ) const
{
    const SPAN ms = spanOf( aSnapped, aAxis );
    const SPAN crossM = spanOf( aSnapped, 1 - aAxis );

    // The winner's ordinate comes first and is drawn unconditionally: it is the alignment the
    // snap actually made, so omitting it would leave a guide that does not explain the move.
    // The snapped box's own two edges follow, and earn a line whenever they land on a neighbor
    // edge as well -- that is what puts a guide down *both* sides of an equal-width neighbor
    // instead of only the side that happened to win by sort order.
    //
    // The moving box's center is deliberately not in this list.  A dashed line down the middle
    // of a symbol is noise, and when the center is what snapped it arrives as aWinnerOrd anyway.
    const int ords[3] = { aWinnerOrd, ms.Min, ms.Max };

    for( int i = 0; i < 3; ++i )
    {
        const int ord = ords[i];

        // Equal-width boxes make ms.Min or ms.Max the winning ordinate, and a zero-size box
        // makes them each other.  One line per ordinate, not one per way of naming it.
        if( i > 0 && std::find( ords, ords + i, ord ) != ords + i )
            continue;

        // Start from the moving box and grow across every neighbor on this ordinate, so three
        // stacked symbols get one line running from the first to the last rather than a stub
        // reaching only the nearest.
        SPAN cross = crossM;
        bool matched = ( i == 0 );

        for( const BOX2I& n : m_neighbors )
        {
            const SPAN ns = spanOf( n, aAxis );

            // Center pairings only for the winning ordinate.  Elsewhere they would draw a line
            // asserting an edge-to-center alignment, which collectAxisCandidates refuses to
            // snap to in the first place.
            if( !( ns.Min == ord || ns.Max == ord || ( i == 0 && ns.Center() == ord ) ) )
                continue;

            const SPAN cs = spanOf( n, 1 - aAxis );
            cross.Min = std::min( cross.Min, cs.Min );
            cross.Max = std::max( cross.Max, cs.Max );
            matched = true;
        }

        if( !matched )
            continue;

        if( aAxis == 0 )
            aResult.Lines.emplace_back( VECTOR2I( ord, cross.Min ), VECTOR2I( ord, cross.Max ) );
        else
            aResult.Lines.emplace_back( VECTOR2I( cross.Min, ord ), VECTOR2I( cross.Max, ord ) );
    }
}


void ALIGNMENT_GUIDE_ENGINE::buildGraphics( const BOX2I& aSnapped, int aAxis,
                                            const SNAP_CANDIDATE&       aWinner,
                                            const std::vector<CLUSTER>& aClusters, int aGridStep,
                                            RESULT&                     aResult ) const
{
    // See the header for what N1/N2 index in each case: the convention differs per kind.
    switch( aWinner.Kind )
    {
    case KIND_ALIGN:
        buildAlignmentLines( aSnapped, aAxis, aWinner.Ord, aResult );
        break;

    case KIND_EQUAL_GAP:
    {
        // Not named far/near: those are legacy Windows macros.
        const CLUSTER& cFar = aClusters[aWinner.N1];
        const CLUSTER& cNear = aClusters[aWinner.N2];

        // The gap the snap was built to reproduce.  Which cluster sits first depends on which
        // side of the pair the moving box landed, and the generator swaps N1/N2 between those
        // two directions, so read the order off the geometry rather than off the indices.
        const int refGap = ( cFar.Max < cNear.Min ) ? ( cNear.Min - cFar.Max )
                                                    : ( cFar.Min - cNear.Max );

        // An exact snap badges only genuinely equal gaps.  A rounded one has to allow the step
        // it could not spend, or the snap it just made would come back unexplained.
        buildGapBadges( aSnapped, aAxis, aClusters, refGap, aWinner.Approx ? aGridStep : 0,
                        aResult );
        break;
    }

    case KIND_BETWEEN:
    {
        const CLUSTER& left = aClusters[aWinner.N1];
        const CLUSTER& right = aClusters[aWinner.N2];
        const SPAN     sMov = spanOf( aSnapped, aAxis );

        const SPAN crossMov = spanOf( aSnapped, 1 - aAxis );
        const int  crossMid = crossMov.Min + crossMov.Size() / 2;

        pushBadge( aResult, aAxis, crossMid, left.Max, sMov.Min, aWinner.Approx );
        pushBadge( aResult, aAxis, crossMid, sMov.Max, right.Min, aWinner.Approx );
        break;
    }

    case KIND_CONTAINER:
    {
        // Centring in a container is an alignment: the moving box's centre lines up with the
        // container's.  So it draws the same kind of line, on that axis only, spanning the
        // container -- which is also the fix for a crosshair that used to appear on both axes
        // when only one had snapped.  Both axes snapping now yields two lines, which reads as a
        // cross without pretending to be one.
        const BOX2I& box = m_containers[aWinner.N1];
        const int    ord = spanOf( box, aAxis ).Center();
        const SPAN   cross = spanOf( box, 1 - aAxis );

        if( aAxis == 0 )
            aResult.Lines.emplace_back( VECTOR2I( ord, cross.Min ), VECTOR2I( ord, cross.Max ) );
        else
            aResult.Lines.emplace_back( VECTOR2I( cross.Min, ord ), VECTOR2I( cross.Max, ord ) );

        break;
    }

    default:
        break;
    }
}


std::optional<ALIGNMENT_GUIDE_ENGINE::RESULT>
ALIGNMENT_GUIDE_ENGINE::FindSnap( const BOX2I& aMoving, int aSnapRange,
                                  const std::optional<VECTOR2I>& aGridStep ) const
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

        const int gridStep = aGridStep ? ( ( axis == 0 ) ? aGridStep->x : aGridStep->y ) : 0;

        collectAxisCandidates( aMoving, axis, clusters[axis], gridStep, candidates );

        std::optional<SNAP_CANDIDATE> best;

        for( const SNAP_CANDIDATE& c : candidates )
        {
            // Dist, not Delta: a rounded fallback has to be judged on how far the cursor is
            // from the spacing it wants, not on where rounding sent it.
            if( std::abs( c.Dist ) > aSnapRange )
                continue;

            if( aGridStep )
            {
                const int g = ( axis == 0 ) ? aGridStep->x : aGridStep->y;

                // Reject, never round: a rounded offset would leave the item off the
                // alignment the guide line is about to claim, or at a spacing the badges
                // say it is not.  Fails closed on a non-positive step, since a missed
                // rejection means a disconnected net.
                if( g <= 0 || c.Delta % g != 0 )
                    continue;
            }

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
            if( !best || std::abs( c.Dist ) < std::abs( best->Dist ) )
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
        {
            const int gridStep = aGridStep ? ( ( axis == 0 ) ? aGridStep->x : aGridStep->y ) : 0;

            buildGraphics( snapped, axis, *winners[axis], clusters[axis], gridStep, result );
        }
    }

    return result;
}
