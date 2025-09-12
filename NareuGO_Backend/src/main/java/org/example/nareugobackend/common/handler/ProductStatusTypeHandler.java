package org.example.nareugobackend.common.handler;


import org.example.nareugobackend.common.model.ProductStatus;
import java.sql.CallableStatement;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import org.apache.ibatis.type.BaseTypeHandler;
import org.apache.ibatis.type.JdbcType;
import org.apache.ibatis.type.MappedJdbcTypes;
import org.apache.ibatis.type.MappedTypes;

@MappedTypes(ProductStatus.class)
@MappedJdbcTypes(JdbcType.VARCHAR)
public class ProductStatusTypeHandler extends BaseTypeHandler<ProductStatus> {

    @Override
    public void setNonNullParameter(PreparedStatement ps, int i, ProductStatus parameter,
        JdbcType jdbcType) throws SQLException {
        ps.setString(i, parameter.name());  // Enum → String
    }

    @Override
    public ProductStatus getNullableResult(ResultSet rs, String columnName) throws SQLException {
        String value = rs.getString(columnName);
        return value != null ? ProductStatus.valueOf(value) : null; // String → Enum
    }

    @Override
    public ProductStatus getNullableResult(ResultSet rs, int columnIndex) throws SQLException {
        String value = rs.getString(columnIndex);
        return value != null ? ProductStatus.valueOf(value) : null;
    }

    @Override
    public ProductStatus getNullableResult(CallableStatement cs, int columnIndex)
        throws SQLException {
        String value = cs.getString(columnIndex);
        return value != null ? ProductStatus.valueOf(value) : null;
    }
}

