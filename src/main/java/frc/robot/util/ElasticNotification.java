// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class ElasticNotification {
    public ElasticNotification() {
    }
    public static void sendNotification(Elastic.Notification notification, String title, String description, Elastic.NotificationLevel level, double displaySeconds) {
        Elastic.sendNotification(notification
        .withLevel(level)
        .withTitle(title)
        .withDescription(description)
        .withDisplaySeconds(displaySeconds)
        );
        Elastic.sendNotification(notification);
    }
}
