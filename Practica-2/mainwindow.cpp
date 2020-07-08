#include "mainwindow.h"
#include "mat2qimage.h"
#include "ui_mainwindow.h"
#include <QTimer>

#include<opencv2/core/core.hpp>
#include<opencv2/ml/ml.hpp>
#include<opencv/cv.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/video/background_segm.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/imgcodecs.hpp>
#include<qfiledialog.h>

#include<QtNetwork>
#include<QDebug>
#include<QMouseEvent>

using namespace cv;


VideoCapture camera(0);

int contadorImagenes = 0;
Mat imagenProcesada;
Mat imagenFija;
//Mat nombre(Y,X, TipoMatriz, Scalar(B,G,R));
Mat error33(600,800,CV_8UC3,Scalar(113,150,255));

bool temporizadorOn = true;
bool abrirCamaraUnicaVez = true;
bool abrirCompuUnicaVez = true;

//---Canales de color----
int canal0Min = 0;
int canal0Max = 0;

int canal1Min = 0;
int canal1Max = 0;

int canal2Min = 0;
int canal2Max = 0;

//coordenadas de el clic
Point clicEtiqueta;
uchar pixel0 = 0;
uchar pixel1 = 0;
uchar pixel2 = 0;

uchar color1_canal0 = 0;
uchar color1_canal1 = 0;
uchar color1_canal2 = 0;

uchar color2_canal0 = 0;
uchar color2_canal1 = 0;
uchar color2_canal2 = 0;

uchar color3_canal0 = 0;
uchar color3_canal1 = 0;
uchar color3_canal2 = 0;

uchar color4_canal0 = 0;
uchar color4_canal1 = 0;
uchar color4_canal2 = 0;


Mat imagenChica;
Mat binariaColor;

Mat imagen;

Point2f puntosOrigen[4];
Point2f puntosDestino[4];
Point2f Figuras[3];

int contador=0;

bool primerVuelta= true;
bool vueltaDer = false;
bool proob = true;

double anterior = 0;

//-------------------------------------------------------------------------------------------

//--Main del programa--(cronometro------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);//Configura la interfaz de usuario

    if(ui->radioButton_3->isChecked()){
        ui->pushButton->setCheckable(true);
       ui->pushButton->setText("Pausar");
    }

   //Creacion de cronometro
     QTimer *cronometro = new QTimer(this);
   //Configurar
     connect(cronometro, SIGNAL(timeout()),this,SLOT(Temporizador()));
   //Iniciar
     cronometro->start(10);
}
//-----------Presionar el mouse----------------------------------------------------------------
void MainWindow::mousePressEvent(QMouseEvent *event){
     int xVentana = event->x()-230;
     int yVentana = event->y();

     if(xVentana >= 0 && yVentana >=0 && xVentana <= 512 && yVentana <= 282){
        clicEtiqueta.x = xVentana;
        clicEtiqueta.y = yVentana;

        qDebug() << "X: " << xVentana << "Y: " << yVentana << endl;
     }
}
//---Temporizador del reloj-----------------------------------------------------------------------------------
void MainWindow::Temporizador(){
   if(temporizadorOn){

       //Capturar una imagen de la camara
       if(ui->radioButton_3->isChecked()){

           if(ui->pushButton->isChecked()){
               //hay que pausar la imagen
               if(imagen.empty()){
                   error33.copyTo(imagen);
               }
           }
           else{
               //la imagen es continua
               if(ui->checkBox->isChecked()){
                   abrirCompuUnicaVez = true;
                   //Mostrar imagen de camara ip
                   QString camaraDireccion = ui->lineEdit->text();
                   QUrl probarDirecion(camaraDireccion);
                   if(probarDirecion.isValid()){
                       //Continuar con el proceso de abrir la camara
                       if(abrirCamaraUnicaVez){
                           if(camera.isOpened()){
                               //libera la camara anterior
                               camera.release();
                           }
                           camera.open(camaraDireccion.toUtf8().constData());
                           if(camera.isOpened()){
                               abrirCamaraUnicaVez = false;
                           }
                       }
                       camera >> imagen;
                       if(imagen.empty()){
                           error33.copyTo(imagen);
                       }
                   }
                   else{
                       //Mostrarle al usuario un error para que conecte la camara
                       error33.copyTo(imagen);
                   }
               }
               else{
                   //Mostarar imagen de camara computadora
                   if(abrirCompuUnicaVez){
                       if(camera.isOpened()){
                           //libera la camara anterior
                           camera.release();
                       }
                       camera.open(0);
                       if(camera.isOpened()){
                           abrirCompuUnicaVez = false;
                       }
                   }

                   camera >> imagen;
                   if(imagen.empty()){
                       error33.copyTo(imagen);
                   }
                   abrirCamaraUnicaVez = true;
               }
               contadorImagenes++;
               ui->lcdNumber->display(contadorImagenes);
           }


       }
       else{
           //Copiar la imagenFija, a la matriz imagen
           //Antes de copiar, asegurar que la matriz,
           //imagenFija, exista.
           if(imagenFija.empty()){
               error33.copyTo(imagen);
           }
           else{
               imagenFija.copyTo(imagen);
           }
       }

       Mat contenedor;
       //Color #FF 96 71
       //        R G  B
       //      Red Green Blue
       //      Rojo Verde Azul
       //      255  150  113

       //Cambiar el tamaño de la imagen que se procesara
       if(!imagen.empty()){
           cv::resize(imagen, imagenChica, Size(512,282),0,0,0);
       }
       else{
           error33.copyTo(imagen);
           cv::resize(imagen, imagenChica, Size(512,282),0,0,0);
       }


       if(ui->radioButton->isChecked()){
           imagenChica.copyTo(imagenProcesada);
           circle(imagenProcesada, clicEtiqueta, 15, Scalar(0,0,255),3,LINE_8,0);

       }

       else if(ui->radioButton_2->isChecked()){
          filtroColores(color1_canal0, color1_canal1, color1_canal2);
          binariaColor.copyTo(imagenProcesada);
       }
       else if(ui->radioButton_5->isChecked()){
           filtroColores(color1_canal0, color1_canal1, color1_canal2);
           Transformacion();
           if(ui->checkBox_2->isChecked()){
              centroObjeto(color2_canal0, color2_canal1, color2_canal2,0);
              centroObjeto(color3_canal0, color3_canal1, color3_canal2,1);
              centroObjeto(color4_canal0, color4_canal1, color4_canal2,2);

                ui->lcdNumber_2->display(Figuras[0].x);
                ui->lcdNumber_3->display(Figuras[0].y);

                ui->lcdNumber_11->display(Figuras[1].x);
                ui->lcdNumber_10->display(Figuras[1].y);

                ui->lcdNumber_13->display(Figuras[2].x);
                ui->lcdNumber_12->display(Figuras[2].y);

                circle(imagenProcesada, Figuras[0], 5, Scalar(0,0,255),2,8,0);
                circle(imagenProcesada, Figuras[1], 5, Scalar(0,0,255),2,8,0);
                circle(imagenProcesada, Figuras[2], 5, Scalar(0,0,255),2,8,0);

                line(imagenProcesada,Figuras[0],Figuras[1],Scalar(255,255,0),2,8,0);
                line(imagenProcesada,Figuras[1],Figuras[2],Scalar(255,255,0),2,8,0);

                double distancia1x = abs(Figuras[0].x-Figuras[1].x);
                double distancia1y = abs(Figuras[0].y-Figuras[1].y);
                double distanciapx1 = sqrt(distancia1x*distancia1x+distancia1y*distancia1y);
                double distancia2x = abs(Figuras[1].x-Figuras[2].x);
                double distancia2y = abs(Figuras[1].y-Figuras[2].y);
                double distanciapx2 = sqrt(distancia2x*distancia2x+distancia2y*distancia2y);
                double distancia3x = abs(Figuras[0].x-Figuras[2].x);
                double distancia3y = abs(Figuras[0].y-Figuras[2].y);
                double distanciapx3 = sqrt(distancia3x*distancia3x+distancia3y*distancia3y);
                double angulo;

                if(distanciapx1+distanciapx2-10>=distanciapx3){

                    angulo = acos((distanciapx3*distanciapx3-distanciapx1*distancia1x-distanciapx2*distanciapx2)/(-2*distanciapx1*distanciapx2));

                    angulo = qRadiansToDegrees(angulo);
                }
                else{
                    angulo = 180;
                }

                ui->lcdNumber_4->display(distanciapx1);
                ui->lcdNumber_7->display(distanciapx2);
                ui->lcdNumber_8->display(angulo);

                if(distanciapx2<60){
                   ui->label_12->setText("Has llegado");
                }
                else{
                   ui->label_12->setText("No has llegado");

                   if(ui->checkBox_3->isChecked()){

                      contador = contador+1;

                      if(primerVuelta && angulo<160){
                          anterior = distanciapx2;
                          conexionServidor("http://robot.local","{\"motor\":\"7\"}");//Derecha
                          primerVuelta = false;
                          proob = true;
                      }

                       if(angulo<160 && contador==34 && anterior < distanciapx2 && proob){
                            contador = 0;
                            vueltaDer = true;
                            proob = false;
                            conexionServidor("http://robot.local","{\"motor\":\"7\"}");//Derecha


                         }
                       else if(angulo<160 && contador==34 && anterior > distanciapx2 && proob){
                           contador = 0;
                           vueltaDer = false;
                           proob = false;
                           conexionServidor("http://robot.local","{\"motor\":\"6\"}");//vuelta izq

                       }

                       else if(angulo<160 && contador==35 && vueltaDer ){
                           contador = 0;
                          conexionServidor("http://robot.local","{\"motor\":\"7\"}");//Derecha
                       }

                       else if(angulo<160 && contador==35 && !vueltaDer ){
                           contador = 0;
                          conexionServidor("http://robot.local","{\"motor\":\"6\"}");//izquierda
                       }
                        else if(angulo>160 && contador==35){
                           contador = 0;
                           conexionServidor("http://robot.local","{\"motor\":\"1\"}");//Adelante
                           primerVuelta = true;
                       }

                     }
                   }
                }
            }


       else{
           error33.copyTo(imagenProcesada);
         }

       //Cambiar el tamaño
       if(!imagenProcesada.empty()){
           cv::resize(imagenProcesada, contenedor, Size(512,288),0,0,INTER_LINEAR);
       }

       //Convertir la imagen Mat a unacontenedor imagen de QT
       QImage imagenQT = Mat2QImage(contenedor);

       //Convertir la imagen de QT a un mapa de pixeles de QT
       QPixmap mapaPixeles = QPixmap::fromImage(imagenQT);

       //Limpiar el contenido de la etiqueta
       ui->label->clear();

       //Mostrar el mapa de pixeles en la etiqueta
       ui->label->setPixmap(mapaPixeles);
   }
}

MainWindow::~MainWindow()
{
    delete ui;
}
//--------Funciones de la interfaz ui------------------------------------------------

void MainWindow::on_pushButton_clicked()
{
   if(ui->radioButton_4->isChecked()){
       temporizadorOn = false;
       QString nombreImagen;
       nombreImagen = QFileDialog::getOpenFileName(this,tr("Abrir imagen"),"/home/riordan/Descargas",tr("Todos los archivos(*.*)"));
       imagenFija = imread(nombreImagen.toUtf8().constData(),1);
       temporizadorOn = true;
   }
   else{
       if(ui->pushButton->isChecked()){
           ui->pushButton->setText("Reanudar");
       }
       else{
           ui->pushButton->setText("Pausar");
       }
   }
}


void MainWindow::on_pushButton_2_clicked()
{
    Mat temporal;
    cvtColor(imagenChica, temporal, CV_BGR2HSV);

    //Filtro Gaussiano
    GaussianBlur(temporal,temporal, Size(23,23),0,0,0);

    //Paso 1.1  leer el valor de el pixel
       Vec3b pixel = temporal.at<Vec3b>(clicEtiqueta.y,clicEtiqueta.x);
       color1_canal0 = pixel.val[0];
       color1_canal1  = pixel.val[1];
       color1_canal2  = pixel.val[2];
}

void MainWindow::on_pushButton_3_clicked()
{
    Mat temporal;
    cvtColor(imagenChica, temporal, CV_BGR2HSV);

    //Filtro Gaussiano
    GaussianBlur(temporal,temporal, Size(23,23),0,0,0);

    //Paso 1.1  leer el valor de el pixel
       Vec3b pixel = temporal.at<Vec3b>(clicEtiqueta.y,clicEtiqueta.x);
       color2_canal0 = pixel.val[0];
       color2_canal1  = pixel.val[1];
       color2_canal2  = pixel.val[2];
}

void MainWindow::on_pushButton_4_clicked()
{
    Mat temporal;
    cvtColor(imagenChica, temporal, CV_BGR2HSV);

    //Filtro Gaussiano
    GaussianBlur(temporal,temporal, Size(23,23),0,0,0);

    //Paso 1.1  leer el valor de el pixel
       Vec3b pixel = temporal.at<Vec3b>(clicEtiqueta.y,clicEtiqueta.x);
       color3_canal0 = pixel.val[0];
       color3_canal1  = pixel.val[1];
       color3_canal2  = pixel.val[2];
}

void MainWindow::on_pushButton_5_clicked()
{
    Mat temporal;
    cvtColor(imagenChica, temporal, CV_BGR2HSV);

    //Filtro Gaussiano
    GaussianBlur(temporal,temporal, Size(23,23),0,0,0);

    //Paso 1.1  leer el valor de el pixel
       Vec3b pixel = temporal.at<Vec3b>(clicEtiqueta.y,clicEtiqueta.x);
       color4_canal0 = pixel.val[0];
       color4_canal1  = pixel.val[1];
       color4_canal2  = pixel.val[2];
}

void MainWindow::on_radioButton_4_clicked(bool checked)
{
    if(checked){
      ui->pushButton->setText("Abrir");
      ui->pushButton->setCheckable(false);
      ui->lineEdit->setEnabled(false);
      ui->checkBox->setEnabled(false);
    }
}

void MainWindow::on_radioButton_3_clicked(bool checked)
{
    if(checked){
      ui->pushButton->setText("Pausar");
      ui->pushButton->setCheckable(true);
      ui->lineEdit->setEnabled(true);
      ui->checkBox->setEnabled(true);
    }
}

void MainWindow::filtroColores(uchar canal0, uchar canal1, uchar canal2){

    //Filtro color 1
    Mat binaria;
    Mat hsv1;

    Point centroContorno;
    //Paso · 1 - Crear una imagen HSV
    cvtColor(imagenChica, hsv1, CV_BGR2HSV);

    //Paso · 1.1 Aplicar filtro gaussiano
    GaussianBlur(hsv1, hsv1, Size(23,23),0,0,0);

    canal0Min = canal0-30;
    canal0Max = canal0+30;

    canal1Min = canal1-30;
    canal1Max = canal1+30;

    canal2Min = canal2-30;
    canal2Max = canal2+30;


    //Paso · 2 - Aplicar un filtro pasa banda
    // a la imagen hsv.
    //inRange(original, Scalar(minimos), Scalar(maximos), destino);
    inRange(hsv1, Scalar(canal0Min, canal1Min, canal2Min), Scalar(canal0Max, canal1Max, canal2Max), binaria);

    //Paso · 3 - Convertir la imagen binaria (C1), imagen BGR
    cvtColor(binaria, binariaColor,CV_GRAY2BGR);

    //Paso · 3.1 encontrar los contornos de la matriz binaria
    Mat copiaBinaria;
    binaria.copyTo(copiaBinaria);

    std::vector<std::vector<Point>> contornos;
    std::vector<Vec4i> jerarquia;

    findContours(copiaBinaria, contornos, jerarquia, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int areaMaxima = 0;

    for (int i = 0; i<(int)contornos.size();i++) {
        int area = contourArea(contornos[i]);

        //Si existen contornos
        if(contornos.size() > 0){

            Moments momentoObjeto1 = moments(contornos[i]);
            centroContorno.x = momentoObjeto1.m10/momentoObjeto1.m00;
            centroContorno.y = momentoObjeto1.m01/momentoObjeto1.m00;

            if( centroContorno.x >= 0 && centroContorno.x <= 256 && centroContorno.y >= 0 && centroContorno.y <= 140){

                if(area > areaMaxima){
                    areaMaxima = area;
                    puntosOrigen[0].x = centroContorno.x;
                    puntosOrigen[0].y = centroContorno.y;
                }
             }
       }
 }

    for (int i = 0; i<(int)contornos.size();i++) {
        int area = contourArea(contornos[i]);

        //Si existen contornos
        if(contornos.size() > 0){

            Moments momentoObjeto1 = moments(contornos[i]);
            centroContorno.x = momentoObjeto1.m10/momentoObjeto1.m00;
            centroContorno.y = momentoObjeto1.m01/momentoObjeto1.m00;

            if( centroContorno.x >= 261 && centroContorno.x <= 512 && centroContorno.y >= 0 && centroContorno.y <= 140){

                if(area > areaMaxima){
                    areaMaxima = area;
                    puntosOrigen[1].x = centroContorno.x;
                    puntosOrigen[1].y = centroContorno.y;
                }
             }
       }
 }

    for (int i = 0; i<(int)contornos.size();i++) {
        int area = contourArea(contornos[i]);

        //Si existen contornos
        if(contornos.size() > 0){

            Moments momentoObjeto1 = moments(contornos[i]);
            centroContorno.x = momentoObjeto1.m10/momentoObjeto1.m00;
            centroContorno.y = momentoObjeto1.m01/momentoObjeto1.m00;

            if( centroContorno.x >= 261 && centroContorno.x <= 512 && centroContorno.y >= 140 && centroContorno.y <= 280){

                if(area > areaMaxima){
                    areaMaxima = area;
                    puntosOrigen[2].x = centroContorno.x;
                    puntosOrigen[2].y = centroContorno.y;
                }
             }
       }
 }

    for (int i = 0; i<(int)contornos.size();i++) {
        int area = contourArea(contornos[i]);

        //Si existen contornos
        if(contornos.size() > 0){

            Moments momentoObjeto1 = moments(contornos[i]);
            centroContorno.x = momentoObjeto1.m10/momentoObjeto1.m00;
            centroContorno.y = momentoObjeto1.m01/momentoObjeto1.m00;

            if( centroContorno.x >= 0 && centroContorno.x <= 256 && centroContorno.y >= 140 && centroContorno.y <= 283){

                if(area > areaMaxima){
                    areaMaxima = area;
                    puntosOrigen[3].x = centroContorno.x;
                    puntosOrigen[3].y = centroContorno.y;
                }
             }
       }
 }


       //Dibujar circulo en el contorno encontrado
       circle(binariaColor, puntosOrigen[0], 10, Scalar(0,0,255),2,8,0);
       circle(binariaColor, puntosOrigen[1], 10, Scalar(0,255,0),2,8,0);
       circle(binariaColor, puntosOrigen[2], 10, Scalar(255,255,0),2,8,0);
       circle(binariaColor, puntosOrigen[3], 10, Scalar(0,255,255),2,8,0);

}



void MainWindow::Transformacion(){

    Mat transformacion;

   Point2f  marco[4];

   marco[0].x=507;
   marco[0].y=5;

   marco[1].x=0;
   marco[1].y=5;

   marco[2].x=0;
   marco[2].y=276;

   marco[3].x=507;
   marco[3].y=276;

    //Esquina superior izquierda
    puntosDestino[0].x = 0;
    puntosDestino[0].y = 0;

    //Esquina superior derecha
    puntosDestino[1].x = 512;
    puntosDestino[1].y = 0;

    //Esquina inferior derecha
    puntosDestino[2].x = 512;
    puntosDestino[2].y = 282;

    //Esquina inferior izquierda
    puntosDestino[3].x = 0;
    puntosDestino[3].y = 282;

    //Transformación
            Mat matrizTransformacion(2,4,CV_32FC1);
            matrizTransformacion = getPerspectiveTransform(puntosOrigen, puntosDestino);
            warpPerspective(imagenChica, transformacion, matrizTransformacion, Size(513,283));

            line(transformacion,marco[0],marco[1],Scalar(255,0,255),5,8,0);
            line(transformacion,marco[1],marco[2],Scalar(255,0,255),5,8,0);
            line(transformacion,marco[2],marco[3],Scalar(255,0,255),5,8,0);
            line(transformacion,marco[3],marco[0],Scalar(255,0,255),5,8,0);

            transformacion.copyTo(imagenProcesada);
}


void MainWindow::centroObjeto(uchar canal0, uchar canal1, uchar canal2,int fig){

    //Filtro color 1
    Mat binaria;
    Mat hsv1;
    Mat binariaColor;
    //Paso · 1 - Crear una imagen HSV
    cvtColor(imagenProcesada, hsv1, CV_BGR2HSV);

    //Paso · 1.1 Aplicar filtro gaussiano
    GaussianBlur(hsv1, hsv1, Size(23,23),0,0,0);

    canal0Min = canal0-20;
    canal0Max = canal0+20;

    canal1Min = canal1-20;
    canal1Max = canal1+20;

    canal2Min = canal2-20;
    canal2Max = canal2+20;


    //Paso · 2 - Aplicar un filtro pasa banda
    // a la imagen hsv.
    //inRange(original, Scalar(minimos), Scalar(maximos), destino);
    inRange(hsv1, Scalar(canal0Min, canal1Min, canal2Min), Scalar(canal0Max, canal1Max, canal2Max), binaria);

    //Paso · 3 - Convertir la imagen binaria (C1), imagen BGR
    cvtColor(binaria, binariaColor,CV_GRAY2BGR);

    //Paso · 3.1 encontrar los contornos de la matriz binaria
    Mat copiaBinaria;
    binaria.copyTo(copiaBinaria);

    std::vector<std::vector<Point>> contornos;
    std::vector<Vec4i> jerarquia;

    findContours(copiaBinaria, contornos, jerarquia, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int areaMaxima = 0;
    int indiceMaximo = 0;
    for (int i = 0; i<(int)contornos.size();i++) {
        int area = contourArea(contornos[i]);
        qDebug() << "Contorno " << i << " : " << area << endl;
        //Encontrar el contorno con mayor area
        if(area > areaMaxima){
            areaMaxima = area;
            indiceMaximo = i;
        }
    }
     Point centroContorno;
    //Si existen contornos
    if(contornos.size() > 0){
        //Encontrar el centro del contorno mayor

        Moments momentoObjeto1 = moments(contornos[indiceMaximo]);
        centroContorno.x = momentoObjeto1.m10/momentoObjeto1.m00;
        centroContorno.y = momentoObjeto1.m01/momentoObjeto1.m00;
    }

        Figuras[fig].x =  centroContorno.x;
        Figuras[fig].y =  centroContorno.y;
}


//------------Conexion con servidor-----------------------------------------------------------
QString MainWindow::conexionServidor(QString url, QString mensaje){

    QString respuestaServidor = "error";

    //Paso 1 - Verificar la red sobre la cual se enviara
    // el mensaje exista. ejecutar comando ifconfig

    QNetworkInterface redConectada = QNetworkInterface::interfaceFromName("wlo1");

    QList<QNetworkAddressEntry> lista = redConectada.addressEntries();

    //Paso 2 - Verificar que la red exista
    if(!lista.empty()){

        QNetworkAddressEntry IPasignada = lista.first();
        qDebug() << "IP Asignada:" << IPasignada.ip() << endl;

        //Paso 3 Crear el Cliente Web
        QNetworkAccessManager *clienteWeb = new QNetworkAccessManager();

        //Paso 4 - Asignar la url del clienteWeb
        QUrl servidor(url.toUtf8().constData());

        //Creacion del mensaje HTML
        QNetworkRequest solicitud;
        QByteArray dimensionMensaje = QByteArray::number(mensaje.size());

        //Paso 5 - Verificar que sea valida la URL
        if(servidor.isValid()){
              respuestaServidor = "Servidor valido";

              //Paso 6 - Formar solicitud
              solicitud.setUrl(servidor);
              solicitud.setRawHeader(QByteArray("user-Agent"),QByteArray("botDAN"));
              solicitud.setRawHeader(QByteArray("Connection"),QByteArray("closed"));
              solicitud.setHeader(QNetworkRequest::ContentTypeHeader,"application/json");
              solicitud.setHeader(QNetworkRequest::ContentLengthHeader, dimensionMensaje);

              // Paso 7 - Enviar la solicitud al servidor
              QNetworkReply *conexionServidor = clienteWeb->post(solicitud,mensaje.toLatin1());

              //Paso 8 - Esperar mientras que el serividor responda
              //QEventLoop funcionLoop;
              //QObject::connect(clienteWeb, SIGNAL(finished(QNetworkReplay*)),&funcionLoop, SLOT(quit()));
              //funcionLoop.exec();

              //Paso 9 - Leer la respuesta del servidor
             // char datosWeb[3000];
              respuestaServidor.clear();
             // int sv=conexionServidor->read(datosWeb,3000);
             // for(int i=0; i<sv;i++){
             //     respuestaServidor+= datosWeb[i];
             // }

        respuestaServidor = "msnina";
     }
    }

    return respuestaServidor;

}
//--------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_6_clicked()
{
   conexionServidor("http://robot.local","{\"motor\":\"1\"}");//Adelante

}

void MainWindow::on_pushButton_7_clicked()
{
   conexionServidor("http://robot.local","{\"motor\":\"2\"}");//Atras

}

void MainWindow::on_pushButton_8_clicked()
{
   conexionServidor("http://robot.local","{\"motor\":\"6\"}");//vuelta izq

}

void MainWindow::on_pushButton_9_clicked()
{
   conexionServidor("http://robot.local","{\"motor\":\"7\"}");//vuelta der

}

void MainWindow::on_pushButton_10_clicked()
{

    conexionServidor("http://robot.local","{\"motor\":\"5\"}");//Paro
    primerVuelta = true;

}

