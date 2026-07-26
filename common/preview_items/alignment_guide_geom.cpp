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

#include <algorithm>

#include <font/font.h>
#include <gal/graphics_abstraction_layer.h>
#include <geometry/geometry_utils.h>
#include <layer_ids.h>
#include <preview_items/item_drawing_utils.h>
#include <preview_items/preview_utils.h>
#include <view/view.h>

using namespace KIGFX;


ALIGNMENT_GUIDE_GEOM::ALIGNMENT_GUIDE_GEOM( const EDA_IU_SCALE& aIuScale ) :
        EDA_ITEM( nullptr, NOT_USED ), // Never added to a BOARD/SCHEMATIC so it needs no type
        m_hasGuides( false ),
        // KiCad's own RED (rgb(132,0,0)) -- the same dark red schematic symbol bodies use.
        // COLOR4D( EDA_COLOR_T ) always comes back fully opaque, which is too heavy for an
        // overlay, so re-apply the previous 0.9 alpha.
        m_color( COLOR4D( RED ).WithAlpha( 0.9 ) ),
        m_iuScale( aIuScale )
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

    // Frontmost, as RULER_ITEM does.  The GAL depth-tests, and everything drawn in one ViewDraw
    // shares a depth unless told otherwise -- which is enough for a filled shape to swallow text
    // drawn immediately after it at the same spot.
    gal.SetLayerDepth( gal.GetMinDepth() );

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

    if( m_guides.Badges.empty() )
        return;

    KIFONT::FONT*                   font = KIFONT::FONT::GetFont();
    const PREVIEW::TEXT_DIMS        textDims = PREVIEW::GetConstantGlyphHeight( &gal );
    const int                       padding = aView->ToWorld( 3 );
    const int                       tick = aView->ToWorld( 4 );

    // A reference string, not the number itself, sets the pill size.  Sizing it to the text
    // makes "8.89" and "10.16" render as visibly different badges, which reads as two kinds of
    // annotation rather than as two numbers.  Longer numbers still grow it instead of
    // overflowing.
    const VECTOR2I refExtents = font->StringBoundaryLimits( wxT( "00.00" ), textDims.GlyphSize,
                                                            textDims.StrokeWidth, false, false,
                                                            KIFONT::METRICS::Default() );

    TEXT_ATTRIBUTES textAttrs;
    textAttrs.m_Size = textDims.GlyphSize;
    textAttrs.m_StrokeWidth = textDims.StrokeWidth;
    textAttrs.m_Halign = GR_TEXT_H_ALIGN_CENTER;
    textAttrs.m_Valign = GR_TEXT_V_ALIGN_CENTER;
    textAttrs.m_Mirrored = gal.IsFlippedX(); // Prevent text mirroring when the view is flipped

    for( const ALIGNMENT_GUIDE_ENGINE::GAP_BADGE& badge : m_guides.Badges )
    {
        // Limitation: always mm, whatever the user's display units are; upgrade path = also
        // take an EDA_UNITS from the frame, as RULER_ITEM does.  The *scale* is not a
        // simplification though -- it comes from the owning editor, or a schematic badge
        // would read 100x small.
        // The tilde-equals prefix is load-bearing, not decoration: a rounded spacing snap gets
        // as close as the grid allows, which can leave the gaps in a run visibly unequal.  An
        // unmarked badge there would read as a claim that they match.
        const wxString text = wxString::Format( badge.Approximate ? wxT( "≈%.2f" )
                                                                  : wxT( "%.2f" ),
                                                badge.Gap / m_iuScale.IU_PER_MM );
        const VECTOR2I extents = font->StringBoundaryLimits( text, textDims.GlyphSize,
                                                             textDims.StrokeWidth, false, false,
                                                             KIFONT::METRICS::Default() );

        const VECTOR2I dir = badge.Vertical ? VECTOR2I( 0, 1 ) : VECTOR2I( 1, 0 );
        const VECTOR2I perp = badge.Vertical ? VECTOR2I( 1, 0 ) : VECTOR2I( 0, 1 );
        const VECTOR2I half = dir * ( badge.Gap / 2 );

        // Half-extents of the label box, sized from the reference string so badges stay the
        // same size whatever their digit count.
        const VECTOR2I halfBox( std::max( extents.x, refExtents.x ) / 2 + padding,
                                std::max( extents.y, refExtents.y ) / 2 + padding );

        gal.SetIsFill( false );
        gal.SetIsStroke( true );
        gal.SetStrokeColor( m_color );

        // A dimension line spanning the measured gap, capped with perpendicular end ticks.
        // Without it the number floats between two symbols with nothing saying which distance
        // it belongs to -- and with equal gaps on screen, that is the whole point.  It stops
        // short of the label rather than running under it, since there is no fill to mask it.
        const VECTOR2I inner = dir * ( badge.Vertical ? halfBox.y : halfBox.x );

        if( ( badge.Gap / 2 ) > ( badge.Vertical ? halfBox.y : halfBox.x ) )
        {
            gal.DrawLine( badge.Pos - half, badge.Pos - inner );
            gal.DrawLine( badge.Pos + inner, badge.Pos + half );
        }

        gal.DrawLine( badge.Pos - half - perp * tick, badge.Pos - half + perp * tick );
        gal.DrawLine( badge.Pos + half - perp * tick, badge.Pos + half + perp * tick );

        // Dashed outline, no fill: the same visual language as the guide lines, and nothing
        // that can hide the number behind it.
        const VECTOR2I tl( badge.Pos.x - halfBox.x, badge.Pos.y - halfBox.y );
        const VECTOR2I tr( badge.Pos.x + halfBox.x, badge.Pos.y - halfBox.y );
        const VECTOR2I br( badge.Pos.x + halfBox.x, badge.Pos.y + halfBox.y );
        const VECTOR2I bl( badge.Pos.x - halfBox.x, badge.Pos.y + halfBox.y );

        DrawDashedLine( gal, SEG( tl, tr ), dashSize );
        DrawDashedLine( gal, SEG( tr, br ), dashSize );
        DrawDashedLine( gal, SEG( br, bl ), dashSize );
        DrawDashedLine( gal, SEG( bl, tl ), dashSize );

        // The guide colour, not a contrasting one: with the fill gone the number sits on the
        // canvas background, so it has to match the lines rather than the vanished pill.
        // Stroke-only, which is what the default stroke font paints with.
        gal.SetIsFill( false );
        gal.SetIsStroke( true );
        gal.SetStrokeColor( m_color );
        font->Draw( &gal, text, badge.Pos, textAttrs, KIFONT::METRICS::Default() );
    }
}
