// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.utils.libraries;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import org.littletonrobotics.junction.LogFileUtil;

/** Utility functions for managing log file paths. */
public final class LogFileUtilEx {

    private LogFileUtilEx() {}

    /** Read the replay log from the environment variable. */
    public static String findReplayLogEnvVar() {
        return invokePrivateLogFileUtilMethod("findReplayLogEnvVar");
    }

    /** Read the replay log from AdvantageScope. */
    public static String findReplayLogAdvantageScope() {
        return invokePrivateLogFileUtilMethod("findReplayLogAdvantageScope");
    }

    @SuppressWarnings("java:S3011")
    private static String invokePrivateLogFileUtilMethod(String methodName) {
        try {
            Method method = LogFileUtil.class.getDeclaredMethod(methodName);
            method.setAccessible(true);
            return (String) method.invoke(null);
        } catch (NoSuchMethodException | IllegalAccessException e) {
            throw new IllegalStateException("Unable to access LogFileUtil." + methodName + "()", e);
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause();
            if (cause instanceof RuntimeException runtimeException) {
                throw runtimeException;
            }
            throw new IllegalStateException("Error while invoking LogFileUtil." + methodName + "()", cause);
        }
    }
}
