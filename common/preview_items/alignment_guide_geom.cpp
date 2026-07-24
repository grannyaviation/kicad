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

#include "preview_items/alignment_guide_geom.h"

#include <font/font.h>
#include <gal/graphics_abstraction_layer.h>
#include <geometry/geometry_utils.h>
#include <layer_ids.h>
#include <preview_items/item_drawing_utils.h>
#include <preview_items/preview_utils.h>
#include <view/view.h>

using namespace KIGFX;


ALIGNMENT_GUIDE_GEOM::ALIGNMENT_GUIDE_GEOM() :
        EDA_ITEM( nullptr, NOT_USED ), // Never added to a BOARD/SCHEMATIC so it needs no type
        m_hasGuides( false ),
        m_color( COLOR4D( 0.9, 0.2, 0.6, 0.9 ) )
{
}


void ALIGNMENT_GUIDE_GEOM::SetGuides( const ALIGNMENT_GUIDE_ENGINE::RESULT& aResult )
{
    m_guides = aResult;
    m_hasGuides = true;
}


void ALIGNMENT_GUIDE_GEOM::ClearGuides()
{
    m_guides = ALIGNMENT_GUIDE_ENGINE::RESULT();
    m_hasGuides = false;
}


const BOX2I ALIGNMENT_GUIDE_GEOM::ViewBBox() const
{
    // Like CONSTRUCTION_GEOM: an edit-time overlay whose extents depend on the view scale
    // (badge sizes), so there is nothing to gain from being precise here.
    BOX2I bbox;
    bbox.SetMaximum();
    return bbox;
}


std::vector<int> ALIGNMENT_GUIDE_GEOM::ViewGetLayers() const
{
    // Don't use LAYER_GP_OVERLAY, we need the guides to be visible on top of the axis cross
    // (same reasoning as CONSTRUCTION_GEOM)
    std::vector<int> layers{ LAYER_UI_START };
    return layers;
}


void ALIGNMENT_GUIDE_GEOM::ViewDraw( int aLayer, VIEW* aView ) const
{
    // Called every frame while dragging; bail before touching the GAL when idle.
    if( !m_hasGuides )
        return;

    GAL&             gal = *aView->GetGAL();
    GAL_SCOPED_ATTRS scopedAttrs( gal, GAL_SCOPED_ATTRS::STROKE_FILL );

    // Everything below is sized with VIEW::ToWorld( pixels ) so it stays constant on screen,
    // as CONSTRUCTION_GEOM does for its dashes and crosses.
    gal.SetIsStroke( true );
    gal.SetIsFill( false );
    gal.SetStrokeColor( m_color );
    gal.SetLineWidth( aView->ToWorld( 1.0 ) );

    const BOX2I viewport = BOX2ISafe( aView->GetViewport() );
    const int   dashSize = aView->ToWorld( 8 );

    for( const SEG& line : m_guides.Lines )
    {
        if( line.A == line.B )
            continue;

        // Guides can run far outside the viewport; DrawDashedLine would emit a dash per
        // ~12 screen px along the whole length, so clip first.
        SEG clipped = line;

        if( ClipLine( &viewport, clipped.A.x, clipped.A.y, clipped.B.x, clipped.B.y ) )
            continue;

        DrawDashedLine( gal, clipped, dashSize );
    }

    for( const VECTOR2I& mark : m_guides.CenterMarks )
    {
        // ponytail: CenterMarks carries no axis, so a single-axis center snap still draws a
        // full crosshair.  Needs an engine change to distinguish; see task notes.
        DrawCross( gal, mark, aView->ToWorld( 16 ) );
    }

    if( m_guides.Badges.empty() )
        return;

    KIFONT::FONT*                   font = KIFONT::FONT::GetFont();
    const PREVIEW::TEXT_DIMS        textDims = PREVIEW::GetConstantGlyphHeight( &gal );
    const int                       padding = aView->ToWorld( 3 );

    TEXT_ATTRIBUTES textAttrs;
    textAttrs.m_Size = textDims.GlyphSize;
    textAttrs.m_StrokeWidth = textDims.StrokeWidth;
    textAttrs.m_Halign = GR_TEXT_H_ALIGN_CENTER;
    textAttrs.m_Valign = GR_TEXT_V_ALIGN_CENTER;
    textAttrs.m_Mirrored = gal.IsFlippedX(); // Prevent text mirroring when the view is flipped

    for( const ALIGNMENT_GUIDE_ENGINE::GAP_BADGE& badge : m_guides.Badges )
    {
        // ponytail: mm hardcoded; upgrade path = pass an EDA_IU_SCALE + EDA_UNITS from the
        // frame when other editors (mils users) come on board.
        const wxString text = wxString::Format( wxT( "%.2f" ), badge.Gap / 1e6 );
        const VECTOR2I extents = font->StringBoundaryLimits( text, textDims.GlyphSize,
                                                             textDims.StrokeWidth, false, false,
                                                             KIFONT::METRICS::Default() );

        // A filled rounded segment as thick as the text is a pill-shaped badge in one call.
        // The round caps supply the horizontal padding.
        const VECTOR2I halfLen( extents.x / 2, 0 );

        gal.SetIsStroke( false );
        gal.SetIsFill( true );
        gal.SetFillColor( m_color );
        gal.DrawSegment( badge.Pos - halfLen, badge.Pos + halfLen, extents.y + 2 * padding );

        // Same trick RULER_ITEM uses for its drop shadows: black on light, white on dark.
        gal.SetIsFill( false );
        gal.SetIsStroke( true );
        gal.SetStrokeColor( PREVIEW::GetShadowColor( m_color ) );
        font->Draw( &gal, text, badge.Pos, textAttrs, KIFONT::METRICS::Default() );
    }
}
