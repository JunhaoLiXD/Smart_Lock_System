const { onValueWritten } = require("firebase-functions/v2/database");
const logger = require("firebase-functions/logger");
const axios = require("axios");

exports.sendPushoverAlert = onValueWritten(
  { ref: "/device/{id}/alarm" },
  async (event) => {
    const newValue = event.data.after.val();

    if (newValue !== true) return;

    const userKey = "APIKEY";
    const appToken = "APPTOKEN";

    await axios.post("https://api.pushover.net/1/messages.json", {
      token: appToken,
      user: userKey,
      message: `Bike alert: unusual activity detected.`,
      title: "Bike Anti-Theft Alert",
      priority: 1,
      sound: "siren",
    });

    logger.info("Pushover sent.");
  }
);
