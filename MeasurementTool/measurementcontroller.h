#ifndef MEASUREMENTCONTROLLER_H
#define MEASUREMENTCONTROLLER_H

#include "serialsession.h"

class MainWindow;

enum class Edge
{
    rising,
    falling,
};

Q_DECLARE_METATYPE(Edge);

class MeasurementController : public QObject
{
    Q_OBJECT
public:
    explicit MeasurementController(MainWindow* view);

signals:
    void instrumentConnected();
    void instrumentFirmwareVersionSet(QString text);
    void instrumentInfoSet(QString text);

    void measurementStarted();
    void measurementFinishedCounting(double frequency, double frequencyError, double period, double periodError);
    void measurementFinishedReciprocal(double frequency, double frequencyError, double period, double periodError, double duty);
    void measurementFinishedPhase(double channelAFrequency, double channelAPeriod, double interval, double phase);
    void measurementTimedOut();

    void pwmFrequencySet(double frequency);

    void status(QString status);
    void errorSignal(QString err);

public slots:
    void doMeasurementCounting(double gateTime);
    void doMeasurementReciprocal();
    void doMeasurementPhase(Edge edge);

    void setPwmFrequency(double frequency);
    void setRelativePhase(double phase);

    void openInterface(QString path);

private:
    enum MeasurementMode { counting, reciprocal };

    bool pollMeasurement(uint8_t* tag_out, uint8_t const** data_out, size_t* length_out);
    void closeInterface();

    bool awaitMeasurementResult(uint8_t which, uint8_t const** reply_payload_out, size_t* reply_length_out);
    bool doMeasurement(uint8_t which, const void* request_data, size_t request_length, void* result_data, size_t result_length);
    bool sendPacketAndAwaitResultCode(uint8_t tag, const uint8_t* data, size_t length, int* rc_out);

    void communicationError();
    void instrumentStateError();
    void error(QString&& error);

    bool checkFirmwareVersion();

    MainWindow* view;
    std::unique_ptr<SerialSession> session;
};

#endif // MEASUREMENTCONTROLLER_H