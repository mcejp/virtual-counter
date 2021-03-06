#ifndef MEASUREMENTCONTROLLER_H
#define MEASUREMENTCONTROLLER_H

#include "guicommon.h"
#include "serialsession.h"

class MainWindow;

enum class Edge
{
    rising,
    falling,
}; Q_DECLARE_METATYPE(Edge);

struct MeasurementOptions
{
    float externalTBError;
    float internalTBError;
}; Q_DECLARE_METATYPE(MeasurementOptions);

class MeasurementController : public QObject
{
    Q_OBJECT
public:
    explicit MeasurementController(MainWindow* view);

    void pleaseAbortMeasurement();

signals:
    void instrumentConnected(InstrumentInfo info);

    void measurementStarted();
    void measurementFinishedCounting(double frequency, double frequencyError, double period, double periodError);
    void measurementFinishedPeriod(double frequency, double frequencyError, double period, double periodError, double duty);
    void measurementFinishedPhase(double channelAFrequency, double channelAPeriod, double interval, double phase);
    void measurementFinishedFreqRatio(double freqRatio, double freqRatioError);
    void measurementTimedOut();

    void didConfigureDigitalGenerators(AllDgenOptions options);

    void status(QString status);
    void errorSignal(QString err);

public slots:
    void doMeasurementCounting(double gateTime);
    void doMeasurementPeriod(unsigned int numPeriods, bool withPhase);
    void doMeasurementPhase(Edge edgeA, Edge edgeB);
    void doMeasurementFreqRatio(unsigned int periods);

    void setMeasurementOptions(MeasurementOptions opts);
    void configureDigitalGenerators(AllDgenOptions options);

    void openInterface(QString path);

private:
    bool pollMeasurement(uint8_t* tag_out, uint8_t const** data_out, size_t* length_out);
    void closeInterface();

    bool awaitMeasurementResult(uint8_t which, uint8_t const** reply_payload_out, size_t* reply_length_out);
    bool doAbortMeasurement(uint8_t which);
    bool doMeasurement(uint8_t which, const void* request_data, size_t request_length, void* result_data, size_t result_length);
    bool sendPacketAndAwaitResultCode(uint8_t tag, const uint8_t* data, size_t length, int* rc_out);

    void communicationError();
    void instrumentStateError();
    void error(QString&& error);

    bool getInstrumentInfo(InstrumentInfo& info_out);
    double getTimebaseRelativeError();

    MainWindow* view;
    std::unique_ptr<SerialSession> session;

    MeasurementOptions options;

    TimebaseSource timebaseSource = TimebaseSource::internal;
    double f_cpu = 1;
};

#endif // MEASUREMENTCONTROLLER_H
