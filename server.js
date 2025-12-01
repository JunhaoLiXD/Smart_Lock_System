import express from "express";
import bodyParser from "body-parser";
import fetch from "node-fetch";

const app = express();
app.use(bodyParser.json());

const USER_KEY = "uajq95f2xr3x9kj4dbgjrnot3cri3z";
const API_TOKEN = "a3zhb5xhpkq1ppzq393tgyriapcqfb";

app.post("/send-alert", async (req, res) => {
  const { title, message } = req.body;

  try {
    const response = await fetch("https://api.pushover.net/1/messages.json", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        token: API_TOKEN,
        user: USER_KEY,
        title,
        message
      })
    });

    const result = await response.json();
    res.json(result);
  } catch (err) {
    console.error(err);
    res.status(500).json({ error: err.message });
  }
});

app.listen(5000, () =>
  console.log("Bike Dashboard backend running on port 5000")
);

