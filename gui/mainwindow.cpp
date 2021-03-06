#include "mainwindow.h"
#include "optionsdialog.h"
#include "ui_mainwindow.h"
#include "ui_optionsdialog.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QtSerialPort/QSerialPortInfo>

#include <cstdio>

#define USE_FORMAT_VALUE_AND_ERROR

constexpr QChar UNICODE_INFINITY = 0x221E;

// PWM measurement range max = f_cpu / MEASUREMENT_PWM_RANGE_COEFF
constexpr int MEASUREMENT_PWM_RANGE_COEFF = 24;

constexpr char OPTIONS_FILE_NAME[] = "options.ini";

constexpr char DISPLAY_VERSION[] = "git_master";

// http://stackoverflow.com/a/13094362
static double round_to_digits(double value, int digits)
{
    if (value == 0.0) // otherwise it will return 'nan' due to the log10() of zero
        return 0.0;

    double factor = pow(10.0, digits - ceil(log10(fabs(value))));
    return round(value * factor) / factor;
}

static unsigned int getNumDecimalDigits(double errorValue)
{
    double magnitude = log10(errorValue);

    if (magnitude < 0)
        return (unsigned int)ceil(-magnitude) + 1;
    else
        return 0;
}

static const char* getFormatWithDecimalDigits(double errorValue)
{
    unsigned int numDecimalDigits = getNumDecimalDigits(errorValue);

    static char buffer[10];
    snprintf(buffer, sizeof(buffer), "%%.%df", numDecimalDigits);
    return buffer;
}

#ifdef USE_FORMAT_VALUE_AND_ERROR
static void formatValueAndError(double value, double error, QString& valueText_out, QString& errorText_out, bool wholeDecimalGroups)
{
    constexpr char DECIMAL_POINT = '.';
    constexpr char THOUSANDS_SEPARATOR = ' ';

    // use a multiple of 3-digit decimal groups for very small numbers
    //constexpr bool DECIMAL_GROUPS = true;

    constexpr int SIGNIFICANT_DIGITS = 1;

    constexpr double CUTOFF_VALUE = 1e-20;
    constexpr int CUTOFF_ORDER = -20;

    // start at the highest digit
    const int value_magnitude = floor(log10(value));
    const int error_magnitude = floor(log10(error));

    int order = std::max(std::max(value_magnitude, error_magnitude), 0);

    //printf("start order: %d, e-m %d\n", order, error_magnitude);

    //int order = floor(err_magnitude) - (SIGNIFICANT_DIGITS - 1);
/*printf("order = %g - 1 = %d\n", floor(err_magnitude), order);
    if (order < -3 && DECIMAL_GROUPS)
        // TODO optimize in int
        order = floor(order / 3.0f) * 3;*/

    static char value_buffer[100];
    static char error_buffer[sizeof(value_buffer)];

    size_t value_buffer_pos = 0;
    size_t error_buffer_pos = 0;

    bool printing_value = false;
    bool printing_error = false;

    for (;;) {
        const double weight = pow(10.0, order);

        const int value_digit = std::min((int)floor(value / weight), 9);
        value -= value_digit * weight;

        const int error_digit = std::min((int)floor(error / weight), 9);
        error -= error_digit * weight;

        //printf("order %d, weight %g, digit %d, rem %g\n", order, weight, value_digit, value);

        if (order <= 0 || value_digit)
            printing_value = true;

        if (order <= 0 || error_digit)
            printing_error = true;

        if (printing_value)
            value_buffer[value_buffer_pos++] = '0' + value_digit;

        if (printing_error)
            error_buffer[error_buffer_pos++] = '0' + error_digit;

        if (!wholeDecimalGroups || order % 3 == 0) {
            //printf("%d <= %d ||  %d <= %d\n", order, CUTOFF_ORDER, order, error_magnitude - SIGNIFICANT_DIGITS + 1);
            if (order <= 0 && (order <= CUTOFF_ORDER || order <= error_magnitude - SIGNIFICANT_DIGITS + 1))
                break;
        }

        if (order == 0) {
            value_buffer[value_buffer_pos++] = DECIMAL_POINT;
            error_buffer[error_buffer_pos++] = DECIMAL_POINT;
        }
        else if (THOUSANDS_SEPARATOR && order % 3 == 0) {
            if (printing_value)
                value_buffer[value_buffer_pos++] = THOUSANDS_SEPARATOR;

            if (printing_error)
                error_buffer[error_buffer_pos++] = THOUSANDS_SEPARATOR;
        }

        --order;
    }

    value_buffer[value_buffer_pos++] = 0;
    error_buffer[error_buffer_pos++] = 0;

    valueText_out = value_buffer;
    errorText_out = error_buffer;
}
#endif

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    pwmOutputPlotView(ipm)
{
    qRegisterMetaType<size_t>("size_t");

    loadOptions(OPTIONS_FILE_NAME);

    ui->setupUi(this);
    measurementPlotView = std::make_unique<MeasurementPlotView>(ui->measurementChart);
    pwmOutputPlotView.init(ui->pwmOutputPlot);
    pwmOutputPlotView.redraw(dgenActual);

    onMeasurementMethodChanged();
    ui->instrumentDeviceLabel->setText("Not connected");
    ui->instrumentFirmwareLabel->setText("--");
    ui->instrumentStatusLabel->setText("--");
    setMeasuredValuesUnknown();

    connect(ui->menuOpenInterface, SIGNAL(triggered(QAction*)), this, SLOT(onOpenInterfaceTriggered(QAction*)));

    QMenu* openInterface = ui->menuOpenInterface;

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo& port : ports) {
        QString name = port.portName();

        if (!port.description().isEmpty())
            name += " (" + port.description() + ")";

        QAction* action = new QAction(name, openInterface);
        action->setObjectName(port.systemLocation());
        openInterface->addAction(action);
    }

    openInterface->setDisabled(ports.size() == 0);

    measurementControllerThread = new QThread;
    measurementController = new MeasurementController(this);
    measurementController->moveToThread(measurementControllerThread);
    connect(measurementControllerThread, SIGNAL (finished()), measurementControllerThread, SLOT (deleteLater()));
    connect(measurementControllerThread, SIGNAL (finished()), measurementController, SLOT (deleteLater()));
    measurementControllerThread->start();

    connect(measurementController, SIGNAL (instrumentConnected(InstrumentInfo)), this, SLOT (onInstrumentConnected(InstrumentInfo)));
    connect(measurementController, SIGNAL (measurementStarted()), this, SLOT (onMeasurementStarted()));
    connect(measurementController, SIGNAL (measurementFinishedCounting(double, double, double, double)),
            this, SLOT (onMeasurementFinishedCounting(double, double, double, double)));
    connect(measurementController, SIGNAL (measurementFinishedPeriod(double, double, double, double, double)),
            this, SLOT (onMeasurementFinishedPeriod(double, double, double, double, double)));
    connect(measurementController, SIGNAL (measurementFinishedPhase(double, double, double, double)),
            this, SLOT (onMeasurementFinishedPhase(double, double, double, double)));
    connect(measurementController, SIGNAL (didConfigureDigitalGenerators(AllDgenOptions)),
            this, SLOT (didConfigureDigitalGenerators(AllDgenOptions)));
    connect(measurementController, SIGNAL (measurementFinishedFreqRatio(double, double)),
            this, SLOT (onMeasurementFinishedFreqRatio(double, double)));
    connect(measurementController, SIGNAL (measurementTimedOut()), this, SLOT (onMeasurementTimedOut()));

    connect(this, SIGNAL (measurementShouldStartCounting(double)), measurementController, SLOT (doMeasurementCounting(double)));
    connect(this, SIGNAL (measurementShouldStartPeriod(unsigned int, bool)), measurementController, SLOT (doMeasurementPeriod(unsigned int, bool)));
    connect(this, SIGNAL (measurementShouldStartPhase(Edge, Edge)), measurementController, SLOT (doMeasurementPhase(Edge, Edge)));
    connect(this, SIGNAL (measurementShouldStartFreqRatio(unsigned int)), measurementController, SLOT (doMeasurementFreqRatio(unsigned int)));
    connect(this, SIGNAL (shouldOpenInterface(QString)), measurementController, SLOT (openInterface(QString)));
    connect(this, SIGNAL (shouldSetMeasurementOptions(MeasurementOptions)), measurementController, SLOT (setMeasurementOptions(MeasurementOptions)));
    connect(this, SIGNAL (shouldConfigureDigitalGenerators(AllDgenOptions)), measurementController, SLOT (configureDigitalGenerators(AllDgenOptions)));

    emit onOptionsUpdated();

    // Auto-connect
    for (const QSerialPortInfo& port : ports) {
        if (port.description() == "Virtual Counter") {
            emit shouldOpenInterface(port.systemLocation());
            break;
        }
    }
}

MainWindow::~MainWindow()
{
    measurementControllerThread->quit();

    measurementPlotView.reset();
    delete ui;
}

void MainWindow::afterMeasurement()
{
    statusString((QString) "Ready.");

    if (continuousMeasurement)
        on_measureButton_clicked();
    else
        ui->measureButton->setEnabled(true);
}

void MainWindow::fade(QLabel* label)
{
    label->setEnabled(false);
}

double MainWindow::getCountingGateTimeSeconds()
{
    static const double values[] = {0.1, 1.0, 10.0};

    int index = ui->measurementGateTimeSelect->currentIndex();

    if (index >= 0 && (size_t) index < sizeof(values) / sizeof(values[0]))
        return values[index];
    else
        return 0.0;
}

int MainWindow::getReciprocalIterations()
{
    static const int values[] = {1, 10, 100, 1000, 10*1000, 100*1000, 1000*1000, 10*1000*1000};

    int index = ui->measurementNumPeriodsSelect->currentIndex();

    if (index >= 0 && (size_t) index < sizeof(values) / sizeof(values[0]))
        return values[index];
    else
        return 1;
}

void MainWindow::loadIpm(QString boardName)
{
    ipm.clear();

    auto dir = QCoreApplication::applicationDirPath();
    auto fileName = dir + "/" + boardName + ".txt";

    QFile inputFile(fileName);
    QVector<QString> tokens;

    if (inputFile.open(QIODevice::ReadOnly)) {
        QTextStream in(&inputFile);

        while (!in.atEnd()) {
            QString line = in.readLine();

            if (line.size() == 0 || line[0] == '#')
                continue;

            auto tokens = line.split('=');

            if (tokens.size() >= 2) {
                ipm.insert(tokens[0], tokens[1]);
            }
        }

        inputFile.close();
    }
    else
        qWarning("Failed to open '%s'", qPrintable(fileName));
}

void MainWindow::loadOptions(QString fileName)
{
    // Pre-load default options
    options.clear();
    options.insert("externalTBError", "50");        // external osc: 50ppm
    options.insert("internalTBError", "20000");     // internal osc: 2 % (20000ppm)

    QFile inputFile(fileName);
    QVector<QString> tokens;

    if (inputFile.open(QIODevice::ReadOnly)) {
        QTextStream in(&inputFile);

        while (!in.atEnd()) {
            QString line = in.readLine();

            if (line.size() == 0 || line[0] == '#')
                continue;

            auto tokens = line.split('=');

            if (tokens.size() >= 2) {
                options.insert(tokens[0], tokens[1]);
            }
        }

        inputFile.close();
    }
    else
        qWarning("Failed to open '%s'", qPrintable(fileName));
}

void MainWindow::onInstrumentConnected(InstrumentInfo info)
{
    this->f_cpu = info.f_cpu;

    loadIpm(info.board);
    pwmOutputPlotView.resetInstrument();

    dgen.setpoint[0] = {true, (float) ui->pwm1FreqSpinner->value(), ui->pwm1DutySlider->value() / 100.0f, 0};
    dgen.setpoint[1] = {true, (float) ui->pwmBFreqSpinner->value(), ui->pwm2DutySlider->value() / 100.0f, (float) ui->pwm2Phase->value()};

    if (dgen.canStartSetting()) {
        emit shouldConfigureDigitalGenerators(dgen.setpoint);
    }

    QString timebaseInfoText;

    switch (info.timebaseSource) {
    case TimebaseSource::external: timebaseInfoText = "external timebase"; break;
    case TimebaseSource::internal: timebaseInfoText = "internal timebase"; break;
    case TimebaseSource::usb20: timebaseInfoText = "USB 2.0 timebase (500ppm)"; break;
    case TimebaseSource::onboardCrystal: timebaseInfoText = "on-board crystal"; break;
    }

    ui->instrumentDeviceLabel->setText("Connected (" + info.port + "), " + timebaseInfoText);
    ui->instrumentFirmwareLabel->setText(info.board + "," + QString::number(info.firmware));

    statusString("Ready.");
    ui->measureButton->setEnabled(true);
    ui->continuousMeasurementToggle->setEnabled(true);

    // refresh ranges etc.
    onMeasurementMethodChanged();
}

void MainWindow::onInstrumentIsAlive()
{
    constexpr char chars[] = {'/', '-', '\\', '|'};
    static int cnt;

    ui->statusBar->showMessage(QString(chars[cnt]));
    cnt = (cnt + 1) % 4;
}

void MainWindow::onInstrumentStatusSet(QString text)
{
    ui->instrumentStatusLabel->setText(text);
}

void MainWindow::onMeasurementFinishedCounting(double frequency, double frequencyError, double period, double periodError)
{
    double timestamp = QDateTime::currentMSecsSinceEpoch() * 1e-3;
    measurementPlotView->addDataPoints(Series::frequency,   &timestamp,    &frequency, &frequencyError,    1);
    measurementPlotView->addDataPoints(Series::period,      &timestamp,    &period,    &periodError,       1);

    setMeasuredValuesFrequencyPeriodDuty(frequency, frequencyError, period, periodError, 0.0);

    showRecommendedFrequencyMeasurementMode(frequency);

    afterMeasurement();
}

void MainWindow::onMeasurementFinishedFreqRatio(double freqRatio, double freqRatioError)
{
    double timestamp = QDateTime::currentMSecsSinceEpoch() * 1e-3;
    measurementPlotView->addDataPoints(Series::freqRatio,   &timestamp,    &freqRatio,  &freqRatioError,    1);

    setMeasuredValuesFreqRatio(freqRatio, freqRatioError);

    afterMeasurement();
}

void MainWindow::onMeasurementFinishedPhase(double channelAFrequency, double channelAPeriod, double interval, double phase)
{
    double timestamp = QDateTime::currentMSecsSinceEpoch() * 1e-3;
    measurementPlotView->addDataPoints(Series::frequency,   &timestamp, &channelAFrequency, nullptr,    1);
    measurementPlotView->addDataPoints(Series::interval,    &timestamp, &interval,  nullptr,    1);
    measurementPlotView->addDataPoints(Series::phase,       &timestamp, &phase,     nullptr,    1);

    setMeasuredValuesFrequencyPeriodIntervalPhase(channelAFrequency, channelAPeriod, interval, phase);

    afterMeasurement();
}

void MainWindow::onMeasurementFinishedPeriod(double frequency, double frequencyError, double period, double periodError, double duty)
{
    double timestamp = QDateTime::currentMSecsSinceEpoch() * 1e-3;
    measurementPlotView->addDataPoints(Series::frequency,   &timestamp,    &frequency, &frequencyError,    1);
    measurementPlotView->addDataPoints(Series::period,      &timestamp,    &period,    &periodError,       1);
    measurementPlotView->addDataPoints(Series::dutyCycle,   &timestamp,    &duty,      nullptr,            1);

    setMeasuredValuesFrequencyPeriodDuty(frequency, frequencyError, period, periodError, duty);

    showRecommendedFrequencyMeasurementMode(frequency);

    afterMeasurement();
}

void MainWindow::onMeasurementMethodChanged()
{
    ui->measurementGateTimeSelect->setEnabled(ui->measurementMethodCounting->isChecked());
    ui->measurementNumPeriodsSelect->setEnabled(ui->measurementMethodPeriod->isChecked() ||
                                                ui->measurementMethodFreqRatio->isChecked());

    // Racks
    if (ui->measurementMethodCounting->isChecked() || ui->measurementMethodPeriod->isChecked() || ui->measurementMethodPwm->isChecked()) {
        ui->frequencyMeasurementRack->show();
        ui->intervalMeasurementRack->hide();
        ui->freqRatioMeasurementRack->hide();
    }
#ifdef ENABLE_MEASUREMENT_PHASE
    else if (ui->measurementMethodInterval->isChecked()) {
        ui->frequencyMeasurementRack->hide();
        ui->intervalMeasurementRack->show();
        ui->freqRatioMeasurementRack->hide();
    }
#endif
    else if (ui->measurementMethodFreqRatio->isChecked()) {
        ui->frequencyMeasurementRack->hide();
        ui->intervalMeasurementRack->hide();
        ui->freqRatioMeasurementRack->show();
    }

    // measurementNumPeriodsSelect
    if (ui->measurementMethodPwm->isChecked()) {
        ui->measurementNumPeriodsSelect->clear();
        ui->measurementNumPeriodsSelect->addItem("1");

        // These were additional options for PWM, but they are now disabled per issue #8
//        ui->measurementNumPeriodsSelect->addItem("10");
//        ui->measurementNumPeriodsSelect->addItem("100");
    }
    else {
        ui->measurementNumPeriodsSelect->clear();
        ui->measurementNumPeriodsSelect->addItem("1");
        ui->measurementNumPeriodsSelect->addItem("10");
        ui->measurementNumPeriodsSelect->addItem("100");
        ui->measurementNumPeriodsSelect->addItem("1 000");
        ui->measurementNumPeriodsSelect->addItem("10 000");

        if (ui->measurementMethodPeriod->isChecked()) {
            ui->measurementNumPeriodsSelect->addItem("100 000");
            ui->measurementNumPeriodsSelect->addItem("1 000 000");
            ui->measurementNumPeriodsSelect->addItem("10 000 000");
        }
    }

#ifdef ENABLE_MEASUREMENT_PHASE
    ui->measurementEdgeSelectA->setEnabled(ui->measurementMethodInterval->isChecked());
    ui->measurementEdgeSelectB->setEnabled(ui->measurementMethodInterval->isChecked());
#endif

    // Port labels (TODO)
    if (ui->measurementMethodCounting->isChecked()) {
        ui->instrumentStatusLabel->setText("Port: " + ipm.value("port.pulse_count"));
    }
    else if (ui->measurementMethodPeriod->isChecked()) {
        ui->instrumentStatusLabel->setText("Port: " + ipm.value("port.period"));
    }
    else if (ui->measurementMethodPwm->isChecked()) {
        ui->instrumentStatusLabel->setText("Ports: " + ipm.value("port.period_pwm_1") + ", " + ipm.value("port.period_pwm_2"));
    }
#ifdef ENABLE_MEASUREMENT_PHASE
    else if (ui->measurementMethodInterval->isChecked()) {
        ui->instrumentStatusLabel->setText("Ports: " + ipm.value("port.interval_a") + ", " + ipm.value("port.interval_b"));
    }
#endif
    else if (ui->measurementMethodFreqRatio->isChecked()) {
        ui->instrumentStatusLabel->setText("Ports: " + ipm.value("port.freq_ratio_a") + ", " + ipm.value("port.freq_ratio_b"));
    }

    ui->recommendedMeasurementModeInfo->setText("");

    // TODO: the following should be wrapped in a single function
    measurementController->pleaseAbortMeasurement();
    if (continuousMeasurement) {
        on_continuousMeasurementToggle_clicked();
    }
    ui->measureButton->setEnabled(true);

    updateMeasurementFrequencyInfo();
}

void MainWindow::onMeasurementStarted()
{
    if (!continuousMeasurement)
        setMeasuredValuesInvalid();

    statusString((QString) "Measurement in progress...");
}

void MainWindow::onMeasurementTimedOut()
{
    statusString((QString) "Measurement timed out!");

    afterMeasurement();
}

void MainWindow::onOpenInterfaceTriggered(QAction* action)
{
    QString path = action->objectName();
    statusString((QString) "Opening " + path);

    emit shouldOpenInterface(path);
}

void MainWindow::onOptionsUpdated()
{
    // Update MeasurementController options
    MeasurementOptions opts;
    opts.externalTBError = options.value("externalTBError").toFloat() * 1e-6;
    opts.internalTBError = options.value("internalTBError").toFloat() * 1e-6;
    emit shouldSetMeasurementOptions(opts);
}

void MainWindow::didConfigureDigitalGenerators(AllDgenOptions options)
{
    ui->pwm1FreqLabel->setText(QString::asprintf("%.2f Hz", options[0].freq));
    ui->pwmADutyLabel->setText(QString::asprintf("%d %%", (int)(100 * options[0].duty)));

    ui->pwm2FreqLabel->setText(QString::asprintf("%.2f Hz", options[1].freq));
    ui->pwm2PhaseLabel->setText(QString::asprintf("%+d °", (int) -options[1].phase));
    ui->pwmBDutyLabel->setText(QString::asprintf("%d %%", (int)(100 * options[1].duty)));

    if (dgen.canContinuePending())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);

    dgenActual = options;
    pwmOutputPlotView.redraw(dgenActual);
}

void MainWindow::saveOptions(QString fileName)
{
    QFile outputFile(fileName);

    if (outputFile.open(QIODevice::WriteOnly)) {
        QTextStream out(&outputFile);

        for (const auto& key : options.keys())      // pretty inefficient, but we don't care
            out << key << "=" << options.value(key) << "\n";

        outputFile.close();
    }
    else
        qWarning("Failed to open '%s'", qPrintable(fileName));
}

void MainWindow::setContinousMeasurement(bool enabled)
{
    continuousMeasurement = enabled;

    if (continuousMeasurement) {
        on_measureButton_clicked();
    }
}

void MainWindow::setMeasuredValuesFrequencyPeriodDuty(double frequency, double frequencyError, double period, double periodError, double duty)
{
    QString frequencyText;
    QString frequencyErrorText;

#ifdef USE_FORMAT_VALUE_AND_ERROR
    formatValueAndError(frequency, frequencyError, frequencyText, frequencyErrorText, true);

    unfade(ui->measuredFreqValue, frequencyText);
    unfade(ui->measuredFreqErrorValue, "+/- " + frequencyErrorText);
#else
    if (frequencyError < 1.0) {
        // 24000000.000
        frequencyText.sprintf("%12.3f", frequency);
    }
    else {
        frequencyText.sprintf("%12.0f", frequency);
    }

    frequencyErrorText.sprintf("+/- %g", round_to_digits(frequencyError, 2));

    unfade(ui->measuredFreqValue, frequencyText);
    unfade(ui->measuredFreqErrorValue, frequencyErrorText);
#endif

    QString periodText;
    QString periodErrorText;

#ifdef USE_FORMAT_VALUE_AND_ERROR
    if (period == INFINITY) {
        periodText = UNICODE_INFINITY;

        unfade(ui->measuredPeriodValue, periodText);
        unfade(ui->measuredPeriodErrorValue, "");
    }
    else {
        formatValueAndError(period, periodError, periodText, periodErrorText, true);

        unfade(ui->measuredPeriodValue, periodText);
        unfade(ui->measuredPeriodErrorValue, "+/- " + periodErrorText);
    }
#else
    if (period == INFINITY) {
        periodText = UNICODE_INFINITY;
    }
    else {
        // FIXME: number formating, number of decimal places
        periodText.sprintf("%11.9f", period);;
    }

    periodErrorText.sprintf("+/- %g", round_to_digits(periodError, 2));

    unfade(ui->measuredPeriodValue, periodText);
    unfade(ui->measuredPeriodErrorValue, periodErrorText);
#endif

    unfade(ui->measuredDutyValue, duty > 0.001 ? QString::number(duty) : QString("N/A"));
}

void MainWindow::setMeasuredValuesFreqRatio(double freqRatio, double freqRatioError)
{
#ifdef USE_FORMAT_VALUE_AND_ERROR
    QString freqRatioText;
    QString freqRatioErrorText;

    formatValueAndError(freqRatio, freqRatioError, freqRatioText, freqRatioErrorText, false);

    unfade(ui->measuredFreqRatioValue, freqRatioText);
    unfade(ui->measuredFreqRatioError, "+/- " + freqRatioErrorText);
#else
    auto format = getFormatWithDecimalDigits(freqRatioError);
    unfade(ui->measuredFreqRatioValue, QString::asprintf(format, freqRatio));
    unfade(ui->measuredFreqRatioError, QString::asprintf("+/- %.2f", freqRatioError));
#endif
}

void MainWindow::setMeasuredValuesInvalid()
{
    fade(ui->measuredFreqValue);
    fade(ui->measuredFreqErrorValue);

    fade(ui->measuredPeriodValue);
    fade(ui->measuredPeriodErrorValue);

    fade(ui->measuredDutyValue);

    fade(ui->channelAFrequency);
    fade(ui->channelAPeriod);
    fade(ui->measuredInterval);
    fade(ui->measuredPhase);

    fade(ui->measuredFreqRatioValue);
    fade(ui->measuredFreqRatioError);
}

void MainWindow::setMeasuredValuesFrequencyPeriodIntervalPhase(double channelAFrequency, double channelAPeriod, double interval, double phase) {
    QString channelAFrequencyText, channelAPeriodText, intervalText, phaseText;
    channelAFrequencyText.sprintf("%12.0f", channelAFrequency);
    channelAPeriodText.sprintf("%11.9f", channelAPeriod);
    intervalText.sprintf("%+11.9f", interval);
    phaseText.sprintf("%+.1f", phase);

    unfade(ui->channelAFrequency, channelAFrequencyText);
    unfade(ui->channelAPeriod, channelAPeriodText);
    unfade(ui->measuredInterval, intervalText);
    unfade(ui->measuredPhase, phaseText);
}

void MainWindow::setMeasuredValuesUnknown()
{
    setMeasuredValuesInvalid();

    ui->measuredFreqValue->setText("?");
    ui->measuredFreqErrorValue->setText("?");

    ui->measuredPeriodValue->setText("?");
    ui->measuredPeriodErrorValue->setText("?");

    ui->measuredDutyValue->setText("?");

    ui->channelAFrequency->setText("?");
    ui->channelAPeriod->setText("?");
    ui->measuredInterval->setText("?");
    ui->measuredPhase->setText("?");

    ui->measuredFreqRatioValue->setText("?");
    ui->measuredFreqRatioError->setText("?");
}

void MainWindow::showRecommendedFrequencyMeasurementMode(double frequency)
{
    QString text;

    if (frequency < 0.01)
        text = "";
    else if (frequency < 10)
        text = "Recommended measurement mode: period (x1)";
    else if (frequency < 100)
        text = "Recommended measurement mode: period (x10)";
    else if (frequency < 1000)
        text = "Recommended measurement mode: period (x100)";
    else if (frequency < 10000)
        text = "Recommended measurement mode: period (x1000)";
    else if (frequency < 100000)
        text = "Recommended measurement mode: period (x10000)";
    else if (frequency < 1000000)
        text = "Recommended measurement mode: period (x100000)";
    else // over 1 MHz
        text = "Recommended measurement mode: period (x1000000)";

    ui->recommendedMeasurementModeInfo->setText(text);
}

void MainWindow::statusString(QString text)
{
    ui->instrumentStatusLabel->setText(text);
}

void MainWindow::updateMeasurementFrequencyInfo()
{
    if (f_cpu <= 0)
        return;

    if (ui->measurementMethodCounting->isChecked()) {
        ui->measurementResolutionInfo->setText(QString::number(1.0 / getCountingGateTimeSeconds()) + " Hz");
        ui->measurementRangeInfo->setText(QString::asprintf("0 - %d Hz", f_cpu / 2));
    }
    else if (ui->measurementMethodPeriod->isChecked()) {
        // FIXME: number formatting, correctnes
        ui->measurementResolutionInfo->setText(QString::number(1000000000.0 / f_cpu / getReciprocalIterations()) + " ns");

        ui->measurementRangeInfo->setText(QString::asprintf("0 - %d Hz", f_cpu / 2));
    }
    else if (ui->measurementMethodPwm->isChecked()) {
        // FIXME: number formatting, correctnes
        ui->measurementResolutionInfo->setText(QString::number(1000000000.0 / f_cpu / getReciprocalIterations()) + " ns");

        ui->measurementRangeInfo->setText(QString::asprintf("0 - %d Hz", f_cpu / MEASUREMENT_PWM_RANGE_COEFF));
    }
}

void MainWindow::unfade(QLabel* label, const QString& text)
{
    label->setEnabled(true);
    label->setText(text);
}

void MainWindow::on_measureButton_clicked()
{
    ui->measureButton->setEnabled(false);

    if (ui->measurementMethodCounting->isChecked()) {
        emit measurementShouldStartCounting(getCountingGateTimeSeconds());
    }
    else if (ui->measurementMethodPeriod->isChecked()) {
        emit measurementShouldStartPeriod(getReciprocalIterations(), false);
    }
    else if (ui->measurementMethodPwm->isChecked()) {
        emit measurementShouldStartPeriod(getReciprocalIterations(), true);
    }
#ifdef ENABLE_MEASUREMENT_PHASE
    else if (ui->measurementMethodInterval->isChecked()) {
        auto edgeA = ui->measurementEdgeSelectA->currentIndex() == 0 ? Edge::rising : Edge::falling;
        auto edgeB = ui->measurementEdgeSelectB->currentIndex() == 0 ? Edge::rising : Edge::falling;
        emit measurementShouldStartPhase(edgeA, edgeB);
    }
#endif
    else if (ui->measurementMethodFreqRatio->isChecked()) {
        emit measurementShouldStartFreqRatio(getReciprocalIterations());
    }
}

void MainWindow::on_measurementGateTimeSelect_currentIndexChanged(int index)
{
    updateMeasurementFrequencyInfo();
}

void MainWindow::on_measurementMethodCounting_toggled(bool checked)
{
    onMeasurementMethodChanged();
}

void MainWindow::on_measurementMethodFreqRatio_toggled(bool checked)
{
    onMeasurementMethodChanged();
}

void MainWindow::on_measurementMethodInterval_toggled(bool checked)
{
    onMeasurementMethodChanged();
}

void MainWindow::on_measurementMethodPeriod_toggled(bool checked)
{
    onMeasurementMethodChanged();
}

void MainWindow::on_measurementMethodPwm_toggled(bool checked)
{
    onMeasurementMethodChanged();
}

void MainWindow::on_actionAbort_measurement_triggered()
{
    measurementController->pleaseAbortMeasurement();

    afterMeasurement();
}

void MainWindow::on_actionQuit_triggered()
{
    this->close();
}

void MainWindow::on_continuousMeasurementToggle_clicked()
{
    setContinousMeasurement(!continuousMeasurement);

    if (continuousMeasurement)
        ui->continuousMeasurementToggle->setText("Stop");
    else
        ui->continuousMeasurementToggle->setText("Run");
}

void MainWindow::on_measurementNumPeriodsSelect_currentIndexChanged(int index)
{
    updateMeasurementFrequencyInfo();
}

void MainWindow::on_measurementPulseWidthEnable_toggled(bool checked)
{
    onMeasurementMethodChanged();
}

void MainWindow::on_menuHelpAbout_triggered()
{
    QMessageBox msgBox;
    msgBox.setIcon(QMessageBox::Icon::Information);
    msgBox.setText(QString::asprintf("Measurement Tool %s\n"
                   "\n"
                   "Developed at Laboratory of Videometry, Department of Measurement, Faculty of Electrical Engineering, Czech Technical University in Prague.\n"
                   "\n"
                   "Copyright (c) 2017-2021 Martin Cejp\n"
                   "\n"
                   "https://github.com/cejpmart/virtual-counter",
                   DISPLAY_VERSION));
    msgBox.setWindowTitle("About Measurement Tool");
    msgBox.exec();
}

void MainWindow::on_menuOptions_triggered()
{
    OptionsDialog dlg;

    auto dlgUi = dlg.getUi();
    dlgUi->externalOscillatorError->setValue(options.value("externalTBError").toInt());
    dlgUi->internalOscillatorError->setValue(options.value("internalTBError").toInt());

    if (dlg.exec()) {
        options.insert("externalTBError", QString::number(dlgUi->externalOscillatorError->value()));
        options.insert("internalTBError", QString::number(dlgUi->internalOscillatorError->value()));

        saveOptions(OPTIONS_FILE_NAME);
        emit onOptionsUpdated();
    }
}

void MainWindow::on_menuSaveCSV_triggered()
{
    if (this->measurementPlotView) {
        auto fileName = QFileDialog::getSaveFileName(this, QString(), QString(), "*.csv");

        if (!fileName.isEmpty())
            this->measurementPlotView->saveSeries(fileName);
    }
}

void MainWindow::on_menuSavePNG_triggered()
{
    if (this->measurementPlotView) {
        auto fileName = QFileDialog::getSaveFileName(this, QString(), QString(), "*.png");

        if (!fileName.isEmpty())
            this->measurementPlotView->savePNG(fileName);
    }
}

void MainWindow::on_plotClear_clicked()
{
    measurementPlotView->clear();
}

void MainWindow::on_plotParamSelect_currentIndexChanged(int index)
{
    if (index == 0) {
        // frequency
        if (this->measurementPlotView)
            this->measurementPlotView->showSeries(Series::frequency);
    }
    else if (index == 1) {
        // period
        if (this->measurementPlotView)
            this->measurementPlotView->showSeries(Series::period);
    }
    else if (index == 2) {
        // duty cycle
        if (this->measurementPlotView)
            this->measurementPlotView->showSeries(Series::dutyCycle);
    }
    else if (index == 3) {
        // phase
        if (this->measurementPlotView)
            this->measurementPlotView->showSeries(Series::phase);
    }
    else if (index == 4) {
        // interval
        if (this->measurementPlotView)
            this->measurementPlotView->showSeries(Series::interval);
    }
    else if (index == 5) {
        // frequency ratio
        if (this->measurementPlotView)
            this->measurementPlotView->showSeries(Series::freqRatio);
    }
}

void MainWindow::on_plotWindowSelect_currentIndexChanged(int index)
{
    switch (index) {
    case 0: this->measurementPlotView->setWindowSeconds(-1); break;
    case 1: this->measurementPlotView->setWindowSeconds(10); break;
    case 2: this->measurementPlotView->setWindowSeconds(60); break;
    }
}

void MainWindow::on_pwmAEnabled_toggled(bool checked)
{
    dgen.setpoint[0].enabled = checked;

    if (dgen.canStartSetting())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);
}

void MainWindow::on_pwmBEnabled_toggled(bool checked)
{
    dgen.setpoint[1].enabled = checked;

    if (dgen.canStartSetting())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);
}

void MainWindow::on_pwmBFreqSpinner_valueChanged(double arg1)
{
    ui->statusBar->showMessage("Value changed. Press Enter to apply.");;
}

void MainWindow::on_pwmBFreqSpinner_editingFinished()
{
    dgen.setpoint[1].freq = ui->pwmBFreqSpinner->value();

    if (dgen.canStartSetting())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);

    ui->statusBar->showMessage("Changes applied.");
}

void MainWindow::on_pwm1DutySlider_valueChanged(int value)
{
    dgen.setpoint[0].duty = value / 100.0f;

    if (dgen.canStartSetting())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);
}

void MainWindow::on_pwm1FreqSpinner_valueChanged(double arg1)
{
    ui->statusBar->showMessage("Value changed. Press Enter to apply.");
}

void MainWindow::on_pwm1FreqSpinner_editingFinished()
{
    dgen.setpoint[0].freq = ui->pwm1FreqSpinner->value();

    if (dgen.canStartSetting())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);

    ui->statusBar->showMessage("Changes applied.");
}

void MainWindow::on_pwm2DutySlider_valueChanged(int value)
{
    dgen.setpoint[1].duty = value / 100.0f;

    if (dgen.canStartSetting())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);
}

void MainWindow::on_pwm2Phase_valueChanged(int value)
{
    dgen.setpoint[1].phase = -value;

    if (dgen.canStartSetting())
        emit shouldConfigureDigitalGenerators(dgen.setpoint);
}
