package build.utils.pathgenerator;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.util.DefaultIndenter;
import com.fasterxml.jackson.core.util.DefaultPrettyPrinter;

public final class PathPlannerPrettyPrinter extends DefaultPrettyPrinter {
    private static final long serialVersionUID = 1L;

    public PathPlannerPrettyPrinter() {
        this._objectFieldValueSeparatorWithSpaces = ": ";
        this.indentArraysWith(DefaultIndenter.SYSTEM_LINEFEED_INSTANCE);
        this.indentObjectsWith(new DefaultIndenter("  ", DefaultIndenter.SYS_LF)); // 2 spaces
    }

    @Override
    public DefaultPrettyPrinter createInstance() {
        return new PathPlannerPrettyPrinter();
    }

    @Override
    public void writeStartArray(JsonGenerator g) throws java.io.IOException {
        if (!_arrayIndenter.isInline()) {
            ++_nesting;
        }
        g.writeRaw('[');
    }

    @Override
    public void writeEndArray(JsonGenerator g, int numValues) throws java.io.IOException {
        if (!_arrayIndenter.isInline()) {
            --_nesting;
        }
        if (numValues > 0) {
            _arrayIndenter.writeIndentation(g, _nesting);
        }

        g.writeRaw(']');
    }
}
