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
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <math/box2.h>
#include <math/vector2d.h>
#include <geometry/seg.h>

/**
 * The frame-agnostic half of align-to-edge.
 *
 * Both the schematic and the symbol editor align a selection the same way -- pick a target, then
 * move every box onto its edge or centre on one axis.  What differs is everything around it:
 * which items are selectable, how a move is committed, and what has to be cleaned up afterwards
 * (the schematic re-trims wires and re-adds junctions; a symbol has neither).  So the arithmetic
 * lives here and the I/O stays in the tools.
 *
 * Pure geometry: no tool, frame or wx dependency, and unit-testable headless.
 */
namespace ALIGN_GEOM
{

enum class MODE
{
    TOP,
    BOTTOM,
    LEFT,
    RIGHT,
    CENTER_X,
    CENTER_Y
};

/**
 * The box the others should line up against: whichever one the cursor is inside, else the first.
 *
 * "Else the first" is not arbitrary -- callers sort by the edge being aligned, so the first box
 * is the extreme one on that axis, which is the sensible default target.
 *
 * @return index into aBoxes, or nullopt when aBoxes is empty
 */
std::optional<size_t> SelectTargetIndex( const std::vector<BOX2I>& aBoxes,
                                         const VECTOR2I&           aCursor );

/**
 * One offset per box, moving each onto aTarget's edge or centre.
 *
 * Every mode moves on a single axis; the other component is always zero.  A box already on the
 * target ordinate gets a zero offset, so passing the target itself is harmless.
 *
 * Takes the target as a box rather than an index into aBoxes, because the caller may align to
 * something that is not in the list being moved -- a locked item is the target precisely because
 * it must not move.
 */
std::vector<VECTOR2I> Deltas( const std::vector<BOX2I>& aBoxes, MODE aMode, const BOX2I& aTarget );

/**
 * The cell of a rectilinear arrangement of segments that contains aPoint.
 *
 * A title block is not made of cells -- it is a rectangle plus a few dividers, and the "box in
 * the corner" a user wants to centre a logo in is only the region those lines enclose.  Each of
 * the four walls is the nearest segment beyond the point that actually spans it on the other
 * axis; a short divider elsewhere in the block must not become a wall.
 *
 * Comparison against aPoint is strict, so a point resting exactly on a divider falls into the
 * cell on one side of it rather than into a zero-width one.  Consequently the returned box always
 * has positive area.
 *
 * @param aSegments axis-aligned segments; diagonals and degenerate points are ignored
 * @return the enclosing cell, or nullopt if aPoint is unbounded on any side
 */
std::optional<BOX2I> CellAt( const std::vector<SEG>& aSegments, const VECTOR2I& aPoint );

} // namespace ALIGN_GEOM
