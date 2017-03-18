package org.iowacityrobotics.y2017;

import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;
import com.jcraft.jsch.UserInfo;

public class SSHUtil {

    private static final JSch jsch = new JSch();
    
    public static Session connect(String host, int port, String user, final String pass) throws JSchException {
        Session sess = jsch.getSession(user, host, port);
        sess.setPassword(pass);
        UserInfo ui = new UserInfo() {

            public String getPassphrase() {
                return null;
            }

            public String getPassword() {
                return pass;
            }

            public boolean promptPassphrase(String arg0) {
                return false;
            }

            public boolean promptPassword(String arg0) {
                return false;
            }

            public boolean promptYesNo(String arg0) {
                return true;
            }

            public void showMessage(String arg0) {
                // NO-OP
            }

        };
        sess.setUserInfo(ui);
        sess.connect();
        return sess;
    }
    
}