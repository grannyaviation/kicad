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

#include <tool/align_geom.h>
#include <tools/sch_tool_base.h>

class SYMBOL_EDIT_FRAME;
class CONDITIONAL_MENU;
class SCH_ITEM;

/**
 * Align a symbol-editor selection to an edge or a centre.
 *
 * Deliberately not SCH_ALIGN_TOOL with a template parameter.  That tool's cleanup re-trims wires,
 * re-adds junctions and runs SCHEMATIC::CleanUp, none of which exists in a symbol, and it commits
 * per item where a symbol editor modifies the whole LIB_SYMBOL once.  The arithmetic they share
 * lives in ALIGN_GEOM.
 */
class SYMBOL_EDITOR_ALIGN_TOOL : public SCH_TOOL_BASE<SYMBOL_EDIT_FRAME>
{
public:
    SYMBOL_EDITOR_ALIGN_TOOL();
    ~SYMBOL_EDITOR_ALIGN_TOOL() override;

    bool Init() override;

    int AlignTop( const TOOL_EVENT& aEvent );
    int AlignBottom( const TOOL_EVENT& aEvent );
    int AlignLeft( const TOOL_EVENT& aEvent );
    int AlignRight( const TOOL_EVENT& aEvent );
    int AlignCenterX( const TOOL_EVENT& aEvent );
    int AlignCenterY( const TOOL_EVENT& aEvent );

private:
    void setTransitions() override;

    /// Shared body of all six.  aUndoLabel is what the undo stack shows.
    int doAlign( ALIGN_GEOM::MODE aMode, const wxString& aUndoLabel );

    /// Twin of SCH_ALIGN_TOOL::adjustDeltaForGrid (eeschema/tools/sch_align_tool.cpp).  Kept as a
    /// separate ~12-line duplicate rather than extracted, because that would mean touching the
    /// schematic tool a second time and it has no automated test coverage.  If this changes, so
    /// should its twin.
    VECTOR2I adjustDeltaForGrid( SCH_ITEM* aItem, const VECTOR2I& aDelta );

    CONDITIONAL_MENU* m_alignMenu = nullptr;
};
