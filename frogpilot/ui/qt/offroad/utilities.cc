#include <QDialog>
#include <QLabel>
#include <QPixmap>
#include <QPointer>
#include <QPushButton>
#include <QUrl>
#include <QVBoxLayout>

#include "frogpilot/ui/qt/offroad/utilities.h"

FrogPilotUtilitiesPanel::FrogPilotUtilitiesPanel(FrogPilotSettingsWindow *parent, bool forceOpen) : FrogPilotListWidget(parent), parent(parent) {
  networkManager = new QNetworkAccessManager(this);
  pairingPollTimer = new QTimer(this);

  forceOpenDescriptions = forceOpen;

  // ── Live Tune Dashboard ────────────────────────────────────────────────────
  ButtonControl *liveTuneBtn = new ButtonControl(
    tr("Live Tune Dashboard"),
    tr("SHOW QR"),
    tr("<b>Open the Live Tune Dashboard on any phone or computer on the same Wi-Fi network.</b> "
       "A QR code will appear — scan it or type the URL shown into any browser.")
  );
  QObject::connect(liveTuneBtn, &ButtonControl::clicked, this, &FrogPilotUtilitiesPanel::showLiveTuneQR);
  if (forceOpenDescriptions) liveTuneBtn->showDescription();
  addItem(liveTuneBtn);

  ParamControl *debugModeToggle = new ParamControl("DebugMode", tr("Debug Mode"), tr("<b>Use all of FrogPilot's developer metrics on your next drive</b> to diagnose issues and improve bug reports."), "");
  if (forceOpenDescriptions) {
    debugModeToggle->showDescription();
  }
  addItem(debugModeToggle);

  ButtonControl *flashPandaButton = new ButtonControl(tr("Flash Panda"), tr("FLASH"), tr("<b>Flash the latest, official firmware onto your Panda device</b> to restore core functionality, fix bugs, or ensure you have the most up-to-date software."));
  QObject::connect(flashPandaButton, &ButtonControl::clicked, [parent, flashPandaButton, this]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to flash the Panda firmware?"), tr("Flash"), this)) {
      std::thread([parent, flashPandaButton, this]() {
        parent->keepScreenOn = true;

        flashPandaButton->setEnabled(false);
        flashPandaButton->setValue(tr("Flashing..."));

        params_memory.putBool("FlashPanda", true);
        while (params_memory.getBool("FlashPanda")) {
          util::sleep_for(UI_FREQ);
        }

        flashPandaButton->setValue(tr("Flashed!"));

        util::sleep_for(2500);

        flashPandaButton->setValue(tr("Rebooting..."));

        util::sleep_for(2500);

        Hardware::reboot();
      }).detach();
    }
  });
  if (forceOpenDescriptions) {
    flashPandaButton->showDescription();
  }
  addItem(flashPandaButton);

  FrogPilotButtonsControl *forceStartedButton = new FrogPilotButtonsControl(tr("Force Drive State"), tr("<b>Force openpilot to be offroad or onroad.</b>"), "", {tr("OFFROAD"), tr("ONROAD"), tr("OFF")}, true);
  QObject::connect(forceStartedButton, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
    if (id == 0) {
      params.putBool("ForceOffroad", true);
      params.putBool("ForceOnroad", false);

      updateFrogPilotToggles();
    } else if (id == 1) {
      params.put("CarParams", params.get("CarParamsPersistent"));
      params.put("FrogPilotCarParams", params.get("FrogPilotCarParamsPersistent"));

      params.putBool("ForceOffroad", false);
      params.putBool("ForceOnroad", true);

      updateFrogPilotToggles();
    } else if (id == 2) {
      params.putBool("ForceOffroad", false);
      params.putBool("ForceOnroad", false);

      updateFrogPilotToggles();
    }
  });
  forceStartedButton->setCheckedButton(2);
  if (forceOpenDescriptions) {
    forceStartedButton->showDescription();
  }
  addItem(forceStartedButton);

  bool paired = params.getBool("PondPaired");
  pondButton = new ButtonControl(
    tr("Pair to \"The Pond\""),
    paired ? tr("UNPAIR") : tr("PAIR"),
    tr("<b>Pair this device with your frogpilot.com account</b> to remotely manage settings from anywhere.")
  );
  QObject::connect(pondButton, &ButtonControl::clicked, [this]() {
    if (!frogpilotUIState()->frogpilot_scene.online) {
      ConfirmationDialog::alert(tr("Please connect to the internet first!"), this);
      return;
    }

    bool isPaired = params.getBool("PondPaired");

    if (isPaired && !ConfirmationDialog::confirm(tr("Are you sure you want to unpair from \"The Pond\"?"), tr("Unpair"), this)) {
      return;
    }

    pondButton->setEnabled(false);
    pondButton->setValue(isPaired ? tr("Unpairing...") : tr("Requesting code..."));

    QJsonObject payload;
    payload["api_token"] = QString::fromStdString(params.get("FrogPilotApiToken"));
    payload["device"] = QString::fromStdString(Hardware::get_name()).trimmed().remove(QChar('\0'));
    payload["frogpilot_dongle_id"] = QString::fromStdString(params.get("FrogPilotDongleId"));

    QString buildMetadataStr = QString::fromStdString(params.get("BuildMetadata"));
    if (!buildMetadataStr.isEmpty()) {
      QJsonDocument bmDoc = QJsonDocument::fromJson(buildMetadataStr.toUtf8());
      if (!bmDoc.isNull()) {
        payload["build_metadata"] = bmDoc.object();
      }
    }

    QNetworkRequest request(QUrl(QString("https://www.frogpilot.com/api/pond/pair/") + (isPaired ? "unpair" : "request")));
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    request.setRawHeader("User-Agent", "frogpilot-api/1.0");

    QByteArray postData = QJsonDocument(payload).toJson(QJsonDocument::Compact);

    QNetworkReply *reply = networkManager->post(request, postData);
    QObject::connect(reply, &QNetworkReply::finished, [this, reply, isPaired, payload]() {
      QByteArray responseBody = reply->readAll();

      reply->deleteLater();

      pondButton->setEnabled(true);
      pondButton->setValue("");

      if (reply->error() != QNetworkReply::NoError) {
        QString errorDetail;
        QString serverError = QJsonDocument::fromJson(responseBody).object().value("error").toString();

        int statusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        if (statusCode == 401) {
          errorDetail = tr("Authentication failed. Please restart your device.");
        } else if (statusCode == 403) {
          errorDetail = serverError.contains("build", Qt::CaseInsensitive)
            ? tr("Unofficial or modified build detected.")
            : tr("Access denied: %1").arg(serverError);
        } else if (statusCode == 429) {
          errorDetail = tr("Too many attempts. Please wait and try again.");
        } else if (statusCode == 503) {
          errorDetail = tr("Server is temporarily unavailable. Please try again later.");
        } else if (statusCode > 0) {
          errorDetail = tr("Server error (%1): %2").arg(statusCode).arg(serverError.isEmpty() ? tr("Unknown error") : serverError);
        } else {
          errorDetail = tr("Network error: %1").arg(reply->errorString());
        }

        if (isPaired) {
          pondButton->setValue(tr("Failed to unpair"));

          QTimer::singleShot(2500, [this]() {
            pondButton->setValue("");
          });
        } else {
          ConfirmationDialog::alert(tr("Failed to get pairing code.\n\n%1").arg(errorDetail), this);
        }
        return;
      }

      if (isPaired) {
        params.putBool("PondPaired", false);

        pondButton->setText(tr("PAIR"));
        pondButton->setValue(tr("Unpaired!"));

        QTimer::singleShot(2500, [this]() {
          pondButton->setValue("");
        });
        return;
      }

      QString code = QJsonDocument::fromJson(responseBody).object().value("code").toString();
      if (code.isEmpty()) {
        ConfirmationDialog::alert(tr("Failed to get pairing code. Please try again."), this);
        return;
      }

      pondButton->setValue(tr("Code: %1").arg(code));

      ConfirmationDialog::alert(tr("Go to \"frogpilot.com/the_pond\" and enter this code: %1").arg(code), this);

      QString dongleId = payload["frogpilot_dongle_id"].toString();
      QString apiToken = payload["api_token"].toString();

      pairingPollTimer->disconnect();

      int pollCount = 0;
      QObject::connect(pairingPollTimer, &QTimer::timeout, [this, code, dongleId, apiToken, pollCount]() mutable {
        if (++pollCount > 200) {
          pairingPollTimer->stop();

          pondButton->setValue(tr("Code expired"));

          QTimer::singleShot(2500, [this]() {
            pondButton->setValue("");
          });
          return;
        }

        QNetworkRequest statusRequest{QUrl(QString("https://www.frogpilot.com/api/pond/pair/status?code=%1&dongle_id=%2&api_token=%3").arg(code, dongleId, apiToken))};
        statusRequest.setRawHeader("User-Agent", "frogpilot-api/1.0");

        QNetworkReply *statusReply = networkManager->get(statusRequest);
        QObject::connect(statusReply, &QNetworkReply::finished, [this, statusReply]() {
          statusReply->deleteLater();
          if (statusReply->error() != QNetworkReply::NoError) {
            return;
          }

          QString status = QJsonDocument::fromJson(statusReply->readAll()).object().value("status").toString();
          if (status == "paired") {
            pairingPollTimer->stop();

            params.putBool("PondPaired", true);

            pondButton->setText(tr("UNPAIR"));
            pondButton->setValue(tr("Paired!"));

            ConfirmationDialog::alert(tr("Device successfully paired to \"The Pond\"!"), this);

            QTimer::singleShot(2500, [this]() {
              pondButton->setValue("");
            });
          } else if (status == "expired") {
            pairingPollTimer->stop();

            pondButton->setValue(tr("Code expired"));

            QTimer::singleShot(2500, [this]() {
              pondButton->setValue("");
            });
          }
        });
      });

      pairingPollTimer->start(3000);
    });
  });
  if (forceOpenDescriptions) {
    pondButton->showDescription();
  }
  addItem(pondButton);
  pondButton->setVisible(QString::fromStdString(params.get("GitRemote")).toLower() == "https://github.com/frogai/openpilot.git");

  ButtonControl *reportIssueButton = new ButtonControl(tr("Report a Bug or an Issue"), tr("REPORT"), tr("<b>Send a bug report</b> so we can help fix the problem!"));
  QObject::connect(reportIssueButton, &ButtonControl::clicked, [this]() {
    if (!frogpilotUIState()->frogpilot_scene.online) {
      ConfirmationDialog::alert(tr("Please connect to the internet before sending a report!"), this);
      return;
    }

    QStringList report_messages = {
      tr("Acceleration feels harsh or jerky"),
      tr("An alert was unclear and I'm not sure what it meant"),
      tr("Braking is too sudden or uncomfortable"),
      tr("I'm not sure if this is normal or a bug:"),
      tr("My steering wheel buttons aren't working"),
      tr("openpilot disengages when I don't expect it"),
      tr("openpilot feels sluggish or slow to respond"),
      tr("Something else (please describe)")
    };

    if (QFile::exists("/data/error_logs/error.txt")) {
      report_messages.prepend(tr("I saw an alert that said \"openpilot crashed\""));
    }

    QString selected_issue = MultiOptionDialog::getSelection(tr("What's going on?"), report_messages, "", this);
    if (selected_issue.isEmpty()) {
      return;
    }

    if (selected_issue.contains("crashed") || selected_issue.contains("not sure") || selected_issue.contains("Something else")) {
      QString extra_input = InputDialog::getText(tr("Please describe what's happening"), this, tr("Send Report"), false, 10, "", 300).trimmed();
      if (extra_input.isEmpty()) {
        return;
      }
      selected_issue += " — " + extra_input;
    }

    QString discord_user = InputDialog::getText(tr("What's your Discord username?"), this, tr("Send Report"), false, -1, QString::fromStdString(params.get("DiscordUsername"))).trimmed();

    QJsonObject reportData;
    reportData["DiscordUser"] = discord_user;
    reportData["Issue"] = selected_issue;

    params.putNonBlocking("DiscordUsername", discord_user.toStdString());
    params_memory.put("IssueReported", QJsonDocument(reportData).toJson(QJsonDocument::Compact).toStdString());

    ConfirmationDialog::alert(tr("Report Sent! Thanks for letting us know!"), this);
  });
  if (forceOpenDescriptions) {
    reportIssueButton->showDescription();
  }
  addItem(reportIssueButton);
  reportIssueButton->setVisible(QString::fromStdString(params.get("GitRemote")).toLower() == "https://github.com/frogai/openpilot.git");

  ButtonControl *resetTogglesButton = new ButtonControl(tr("Reset Toggles to Default"), tr("RESET"), tr("<b>Reset all toggles to their default values.</b>"));
  QObject::connect(resetTogglesButton, &ButtonControl::clicked, [parent, resetTogglesButton, this]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset all toggles to their default values?"), tr("Reset"), this)) {
      std::thread([parent, resetTogglesButton, this]() {
        parent->keepScreenOn = true;

        resetTogglesButton->setEnabled(false);
        resetTogglesButton->setValue(tr("Resetting..."));

        std::vector<std::string> all_keys = params.allKeys();
        for (const std::string &key : all_keys) {
          if (excluded_keys.count(key)) {
            continue;
          }
          std::optional<std::string> default_value = params.getKeyDefaultValue(key);
          if (default_value.has_value()) {
            params.put(key, default_value.value());
          }
        }

        updateFrogPilotToggles();

        resetTogglesButton->setValue(tr("Reset!"));

        util::sleep_for(2500);

        resetTogglesButton->setValue("");
      }).detach();
    }
  });
  if (forceOpenDescriptions) {
    resetTogglesButton->showDescription();
  }
  addItem(resetTogglesButton);

  ButtonControl *resetTogglesButtonStock = new ButtonControl(tr("Reset Toggles to Stock openpilot"), tr("RESET"), tr("<b>Reset all toggles to match stock openpilot.</b>"));
  QObject::connect(resetTogglesButtonStock, &ButtonControl::clicked, [parent, resetTogglesButtonStock, this]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset all toggles to match stock openpilot?"), tr("Reset"), this)) {
      std::thread([parent, resetTogglesButtonStock, this]() {
        parent->keepScreenOn = true;

        resetTogglesButtonStock->setEnabled(false);
        resetTogglesButtonStock->setValue(tr("Resetting..."));

        std::vector<std::string> all_keys = params.allKeys();
        for (const std::string &key : all_keys) {
          if (excluded_keys.count(key)) {
            continue;
          }
          std::optional<std::string> stock_value = params.getStockValue(key);
          if (stock_value.has_value()) {
            params.put(key, stock_value.value());
          }
        }

        updateFrogPilotToggles();

        resetTogglesButtonStock->setValue(tr("Reset!"));

        util::sleep_for(2500);

        resetTogglesButtonStock->setValue("");
      }).detach();
    }
  });
  if (forceOpenDescriptions) {
    resetTogglesButtonStock->showDescription();
  }
  addItem(resetTogglesButtonStock);
}

void FrogPilotUtilitiesPanel::showEvent(QShowEvent *event) {
  FrogPilotListWidget::showEvent(event);

  bool isPaired = params.getBool("PondPaired");
  pondButton->setText(isPaired ? tr("UNPAIR") : tr("PAIR"));
}

void FrogPilotUtilitiesPanel::showLiveTuneQR() {
  QString ip = frogpilotUIState()->wifi->getIp4Address();
  if (ip.isEmpty()) {
    ConfirmationDialog::alert(tr("Device is not connected to Wi-Fi.\nConnect to Wi-Fi first, then try again."), this);
    return;
  }

  QString url = QString("http://%1:8765").arg(ip);

  QDialog *dialog = new QDialog(this);
  dialog->setModal(true);
  dialog->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
  dialog->setStyleSheet("QDialog { background-color: #1B1B1B; border-radius: 30px; }");
  dialog->setFixedSize(680, 780);

  QVBoxLayout *layout = new QVBoxLayout(dialog);
  layout->setContentsMargins(40, 36, 40, 36);
  layout->setSpacing(16);

  QLabel *title = new QLabel(tr("Live Tune Dashboard"));
  title->setAlignment(Qt::AlignCenter);
  title->setStyleSheet("color: white; font-size: 46px; font-weight: 600;");
  layout->addWidget(title);

  // QR image — starts as a loading placeholder, filled once the network reply arrives
  QLabel *qrLabel = new QLabel(tr("Fetching QR code\u2026"));
  qrLabel->setAlignment(Qt::AlignCenter);
  qrLabel->setFixedSize(480, 480);
  qrLabel->setWordWrap(true);
  qrLabel->setStyleSheet("color: #8b949e; font-size: 28px; background: #2d2d2d; border-radius: 12px;");
  layout->addWidget(qrLabel, 0, Qt::AlignCenter);

  QLabel *urlLabel = new QLabel(url);
  urlLabel->setAlignment(Qt::AlignCenter);
  urlLabel->setStyleSheet("color: #58a6ff; font-size: 38px; font-weight: 700;");
  layout->addWidget(urlLabel);

  QLabel *hint = new QLabel(tr("Scan with your phone or type the URL above into any browser on the same Wi-Fi network"));
  hint->setAlignment(Qt::AlignCenter);
  hint->setWordWrap(true);
  hint->setStyleSheet("color: #8b949e; font-size: 24px;");
  layout->addWidget(hint);

  QPushButton *closeBtn = new QPushButton(tr("Close"));
  closeBtn->setStyleSheet(
    "QPushButton { background: #333; color: white; border: none; border-radius: 10px;"
    "  padding: 16px 60px; font-size: 34px; font-weight: 600; }"
    "QPushButton:pressed { background: #444; }"
  );
  QObject::connect(closeBtn, &QPushButton::clicked, dialog, &QDialog::accept);
  layout->addWidget(closeBtn, 0, Qt::AlignCenter);

  // Fetch QR code PNG from api.qrserver.com (480×480, no border)
  QString encodedUrl = QString(QUrl::toPercentEncoding(url));
  QString qrApiUrl = QString("https://api.qrserver.com/v1/create-qr-code/?size=480x480&margin=1&data=%1").arg(encodedUrl);
  QNetworkReply *reply = networkManager->get(QNetworkRequest(QUrl(qrApiUrl)));

  QPointer<QLabel> safeLabel(qrLabel);
  QObject::connect(reply, &QNetworkReply::finished, [reply, safeLabel]() {
    reply->deleteLater();
    if (!safeLabel) return;
    if (reply->error() != QNetworkReply::NoError) {
      safeLabel->setText(QObject::tr("Could not fetch QR code.\nType the URL above into your browser."));
      return;
    }
    QPixmap px;
    if (px.loadFromData(reply->readAll()) && !px.isNull()) {
      safeLabel->setPixmap(px.scaled(480, 480, Qt::KeepAspectRatio, Qt::SmoothTransformation));
      safeLabel->setStyleSheet("background: white; border-radius: 12px; padding: 4px;");
    }
  });

  dialog->exec();

  // Abort any in-flight request if dialog was closed before image arrived
  if (!reply->isFinished()) reply->abort();
  delete dialog;
}
