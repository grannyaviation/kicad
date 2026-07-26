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

#include <vector>

#include <base_units.h>
#include <eda_item.h>
#include <gal/color4d.h>
#include <tool/alignment_guide_engine.h>

namespace KIGFX
{

/**
 * Preview item that renders smart alignment guides: dashed guide lines,
 * equal-spacing distance badges and center marks, from an
 * ALIGNMENT_GUIDE_ENGINE::RESULT.
 */
class ALIGNMENT_GUIDE_GEOM : public EDA_ITEM
{
public:
    /**
     * @param aIuScale the internal-unit scale of the editor that owns this item; the gap
     *                 badges are meaningless without it (1e6 IU/mm on a board, 1e4 in the
     *                 schematic).  No default on purpose -- a wrong scale is a silently
     *                 wrong number on screen, so every owner has to name its own.
     */
    ALIGNMENT_GUIDE_GEOM( const EDA_IU_SCALE& aIuScale );

    wxString GetClass() const override { return wxT( "ALIGNMENT_GUIDE_GEOM" ); }

    void SetGuides( const ALIGNMENT_GUIDE_ENGINE::RESULT& aResult );
    void ClearGuides();
    bool HasGuides() const { return m_hasGuides; }

    /**
     * Warning glyphs, one per position, for items that cannot sit on the working grid.
     *
     * Kept apart from the guides rather than folded into RESULT: being off grid is a property
     * of the item, not of any alignment, so it must survive ClearGuides() -- which fires
     * mid-drag whenever an axis lock overrides the cursor.
     */
    void SetOffGridWarnings( std::vector<VECTOR2I> aPositions );
    bool HasOffGridWarnings() const { return !m_offGrid.empty(); }

    void SetColor( const COLOR4D& aColor ) { m_color = aColor; }

    const BOX2I ViewBBox() const override;

    void ViewDraw( int aLayer, VIEW* aView ) const override;

    std::vector<int> ViewGetLayers() const override;

#if defined( DEBUG )
    void Show( int nestLevel, std::ostream& os ) const override {}
#endif

private:
    ALIGNMENT_GUIDE_ENGINE::RESULT m_guides;
    bool                           m_hasGuides;
    std::vector<VECTOR2I>          m_offGrid;
    COLOR4D                        m_color;
    COLOR4D                        m_warningColor;
    const EDA_IU_SCALE&            m_iuScale;
};

} // namespace KIGFX
