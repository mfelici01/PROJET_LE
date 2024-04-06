#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QScrollBar>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Serial");

    mSerial = new QSerialPort(this);
    updateSerialPorts();

    mSerialScanTimer = new QTimer(this);
    mSerialScanTimer->setInterval(5000);
    mSerialScanTimer->start();

    connect(mSerialScanTimer, &QTimer::timeout,
            this, &MainWindow::updateSerialPorts);

    connect(ui->lineEdit, &QLineEdit::returnPressed,
            this, &MainWindow::on_sendPushButton_clicked);

    connect(mSerial, &QSerialPort::readyRead,
            this, &MainWindow::serialReadyRead);

    /* The unchanged code is omitted */

    mData = QSharedPointer<QCPGraphDataContainer>(new QCPGraphDataContainer);

    /* Setup plot */
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->plot->legend->setVisible(true);
    QFont legendFont = font();
    legendFont.setPointSize(10);
    ui->plot->legend->setFont(legendFont);
    ui->plot->legend->setSelectedFont(legendFont);
    ui->plot->legend->setSelectableParts(QCPLegend::spItems);
    ui->plot->yAxis->setLabel("Pressure");
    ui->plot->xAxis->setLabel("Temperature [degC]");
    ui->plot->clearGraphs();
    ui->plot->addGraph();

    ui->plot->graph()->setPen(QPen(Qt::black));
    ui->plot->graph()->setData(mData);
    ui->plot->graph()->setName("STM32 ADC");

}

void MainWindow::updateSerialPorts()
{
    mSerialPorts = QSerialPortInfo::availablePorts();

    ui->comboBox->clear();
    for (QSerialPortInfo port : mSerialPorts) {
        ui->comboBox->addItem(port.portName(), port.systemLocation());
    }
}

void MainWindow::on_connectPushButton_clicked()
{
    ui->connectPushButton->setEnabled(false);
    QString serialLoc  =  ui->comboBox->currentData().toString();

    if (mSerial->isOpen()) {
        qDebug() << "Serial already connected, disconnecting!";
        mSerial->close();
    }

    mSerial->setPortName(serialLoc);
    mSerial->setBaudRate(QSerialPort::Baud115200);
    mSerial->setDataBits(QSerialPort::Data8);
    mSerial->setParity(QSerialPort::NoParity);
    mSerial->setStopBits(QSerialPort::OneStop);
    mSerial->setFlowControl(QSerialPort::NoFlowControl);

    if(mSerial->open(QIODevice::ReadWrite)) {
        qDebug() << "SERIAL: OK!";
    } else {
        qDebug() << "SERIAL: ERROR!";
    }
    ui->connectPushButton->setEnabled(true);
}

void MainWindow::on_sendPushButton_clicked()
{
    if (mSerial->isOpen()) {

        QString str= ui->lineEdit->text();
        ui->lineEdit->clear();
        str.append("\r\n");
        mSerial->write(str.toLocal8Bit());
    } else {
        qDebug() << "Serial port not connected!";
    }
}

void MainWindow::serialReadyRead()
{
    QByteArray data = mSerial->readAll();
    QString str = QString(data);
    ui->textBrowser->insertPlainText(str);
    QScrollBar *sb = ui->textBrowser->verticalScrollBar();
    sb->setValue(sb->maximum());

    QStringList lines = str.split("\n", Qt::SkipEmptyParts);
    for (const QString &line : lines) {
        if (line.startsWith("Temperature [degC]:", Qt::CaseInsensitive)) {
            QStringList parts = line.split(" ");
            if (parts.size() == 3) {
                double temperature = parts.at(2).toDouble();
                qDebug() << "Got a Temperature [degC]: " << temperature;
                mData->add(QCPGraphData(temperature,mData->size()));
            }
        } else if (line.startsWith("Pressure [hPa]:", Qt::CaseInsensitive)) {
            QStringList parts = line.split(" ");
            if (parts.size() == 3) {
                double pressure = parts.at(2).toDouble();
                qDebug() << "Got a Pressure [hPa]: " << pressure;
                mData->add(QCPGraphData(mData->size(), pressure));
            }
        }

    }


    // Rescale axes and replot after adding both temperature and pressure data
    ui->plot->rescaleAxes();
    ui->plot->replot();
}

void MainWindow::on_clearGraphPushButton_clicked()
{
    mData->clear();
    ui->plot->rescaleAxes();
    ui->plot->replot();
}

void MainWindow::on_saveGraphPushButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save pdf"), "",
                                                    tr("Pdf files (*.pdf)"));

    if (!filename.isEmpty()) {
        ui->plot->savePdf(filename);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
