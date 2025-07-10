package com.example.catvital.MySQLite;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.Timestamp;

public class MySQLConnections {

    public final static String TAG = "MySql";
    private static MySQLConnections instance = null;

    private MySQLConnections() {
        System.out.println("正在初始化MySQL数据库连接...");
        System.out.println("连接地址: " + dbURL);
    }

    public static synchronized MySQLConnections getInstance() {
        if (instance == null) {
            instance = new MySQLConnections();
        }
        return instance;
    }

    // 修改后的方法（移除了static）
    public Connection getConnection() throws SQLException {
        try {
            Class.forName(driver);
            Connection conn = DriverManager.getConnection(dbURL, user, password);
            System.out.println("数据库连接成功！");
            return conn;
        } catch (ClassNotFoundException e) {
            System.err.println("错误：找不到MySQL JDBC驱动！");
            throw new SQLException("找不到JDBC驱动", e);
        }
    }

    public boolean testConnection() {
        System.out.println("正在测试数据库连接...");
        try (Connection conn = getConnection()) {
            if (conn != null && !conn.isClosed()) {
                System.out.println("数据库连接测试通过");
                return true;
            }
            return false;
        } catch (SQLException e) {
            System.err.println("数据库连接测试失败: " + e.getMessage());
            return false;
        }
    }

    public static void main(String[] args) {
        MySQLConnections dbConnector = MySQLConnections.getInstance();

        if (dbConnector.testConnection()) {
            System.out.println("数据库连接状态: 正常");
        } else {
            System.out.println("数据库连接状态: 异常");
        }
    }

    /**
     * 获取指定表的最新更新时间
     * @param tableName 要查询的表名
     * @return 该表的最新更新时间，如果没有记录则返回null
     * @throws SQLException
     */
    public Timestamp getTableLastUpdateTime(String tableName) throws SQLException {
        try (
                Connection conn = getConnection();
             PreparedStatement pstmt = conn.prepareStatement(
                     "SELECT UPDATE_TIME FROM information_schema.tables " +
                             "WHERE TABLE_SCHEMA = 'cat' AND TABLE_NAME = ?")) {

            pstmt.setString(1, tableName);
            try (ResultSet rs = pstmt.executeQuery()) {
                if (rs.next()) {
                    return rs.getTimestamp("UPDATE_TIME");
                }
                return null;
            }
        }
    }
}