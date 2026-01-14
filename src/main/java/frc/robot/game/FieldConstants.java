// REBELLION 10014

package frc.robot.game;

import frc.robot.game.field.AndyMarkField;
import frc.robot.game.field.FieldDimensions;
import frc.robot.game.field.WeldedField;

public class FieldConstants {
    // ALL OF THESE VALUES ARE ASSUMED TO BE FROM 🔵 BLUE SIDE 🔵
    // ALL VALUES ARE FROM: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // IF NOT INCLUDED THERE, THEY ARE FROM THE CAD: https://cad.onshape.com/documents/8a691e28680da30504859fce/w/c6aa636fb23edb3f1e272fb1/e/5e2b2310531e01f25fd97afd

    public enum FieldType {
        ANDYMARK,
        WELDED
    }

    public static final FieldType kFieldType = FieldType.ANDYMARK;
    public static final FieldDimensions kField = kFieldType == FieldType.ANDYMARK ? new AndyMarkField() : new WeldedField();
}
