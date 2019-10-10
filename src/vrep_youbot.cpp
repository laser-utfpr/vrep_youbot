#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "turtlesim/Pose.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include "FLIE-master/flie.h"

using namespace std;

//robô
geometry_msgs::Point feedback;
double roll, pitch, yaw;
//posições da ponta do braço
float pos_x, pos_y, pos_z;
//posição do cubo na imagem
float cubo_x, cubo_y;

//linear
double lEntrada, lSaida;
double lUltimoErro = 99;

//angular
double aEntrada, aSaida;
double aUltimoErro = 99;

//FUZZY
fuzzy_control fuzzyAngularBase, fuzzyLinearBase;
linguisticvariable erroAngular, erroLinear, vAngular, vLinear;
fuzzy_set catErroAngular[7];
fuzzy_set catErroLinear[3];
fuzzy_set catVAngular[5];
fuzzy_set catVLinear[4];
rule regras[15];

//mesa
double pos_desejada_youbot[2];
double pos_em_frente_da_mesa[2] = {1.5,0.9};
double yaw_em_frente_da_mesa = M_PI/2;

void preparaControleFuzzyBase(){

	/* REGRAS FUZZY */
	/* A ideia é priorizar a angular para posteriormente acelerar, tentando evitar "grandes arcos"
	*  desnecessários. O objetivo é fazer com que o robô mova-se sem sair das bordas do mapa, com
	*  liberdade de movimento de -1.5,-1.5 a 1.5,1.5 com a garantia de que não cairá para fora do mapa
	*  com movimentos iniciados dentro da área delimitada por essas coordenadas.
	*/

	//Erro Angular

	catErroAngular[0].setname("AltoNeg");
	catErroAngular[0].setrange(-3.15, 3.15);
	catErroAngular[0].setval(-3.15, -3.15, -1);

	catErroAngular[1].setname("BaixoNeg");
	catErroAngular[1].setrange(-3.15, 3.15);
	catErroAngular[1].setval(-1.5, -0.75, -0.05);

	catErroAngular[2].setname("Reto");
	catErroAngular[2].setrange(-3.15, 3.15);
	catErroAngular[2].setval(-0.2, 0, 0.2);

	catErroAngular[3].setname("BaixoPos");
	catErroAngular[3].setrange(-3.15, 3.15);
	catErroAngular[3].setval(0.05, 0.75, 1.5);

	catErroAngular[4].setname("AltoPos");
	catErroAngular[4].setrange(-3.15, 3.15);
	catErroAngular[4].setval(1, 3.15, 3.15);

	erroAngular.setname("ErroAngular");

	erroAngular.includecategory(&catErroAngular[0]);
	erroAngular.includecategory(&catErroAngular[1]);
	erroAngular.includecategory(&catErroAngular[2]);
	erroAngular.includecategory(&catErroAngular[3]);
	erroAngular.includecategory(&catErroAngular[4]);

	//Erro Linear
	catErroLinear[0].setname("Chegou");
	catErroLinear[0].setrange(0, 7.1);
	catErroLinear[0].setval(0, 0, 0.1);

	catErroLinear[1].setname("Perto");
	catErroLinear[1].setrange(0, 7.1);
	catErroLinear[1].setval(0, 0.5, 1.5);

	catErroLinear[2].setname("Longe");
	catErroLinear[2].setrange(0, 7.1);
	catErroLinear[2].setval(0.9, 6, 7.1, 7.1);

	erroLinear.setname("ErroLinear");

	erroLinear.includecategory(&catErroLinear[0]);
	erroLinear.includecategory(&catErroLinear[1]);
	erroLinear.includecategory(&catErroLinear[2]);
	
	//Velocidade Angular
	catVAngular[0].setname("ForteEsq");
	catVAngular[0].setrange(-1, 1);
	catVAngular[0].setval(-1, -1, -0.5);

	catVAngular[1].setname("Esq");
	catVAngular[1].setrange(-1, 1);
	catVAngular[1].setval(-1, -0.4, 0);

	catVAngular[2].setname("Reto");
	catVAngular[2].setrange(-1, 1);
	catVAngular[2].setval(-0.3, 0, 0.3);

	catVAngular[3].setname("Dir");
	catVAngular[3].setrange(-1, 1);
	catVAngular[3].setval(0, 0.4, 1);

	catVAngular[4].setname("ForteDir");
	catVAngular[4].setrange(-1, 1);
	catVAngular[4].setval(0.5, 1, 1);

	
	vAngular.setname("Vel. Angular");

	vAngular.includecategory(&catVAngular[0]);
	vAngular.includecategory(&catVAngular[1]);
	vAngular.includecategory(&catVAngular[2]);
	vAngular.includecategory(&catVAngular[3]);
	vAngular.includecategory(&catVAngular[4]);

	//Velocidade Linear
	catVLinear[0].setname("Parado");
	catVLinear[0].setrange(0, 1);
	catVLinear[0].setval(0, 0, 0.2);

	catVLinear[1].setname("Medio");
	catVLinear[1].setrange(0, 1);
	catVLinear[1].setval(0, 0.1, 1);

	catVLinear[2].setname("Rapido");
	catVLinear[2].setrange(0, 1);
	catVLinear[2].setval(0.9, 1, 1);

	vLinear.setname("Vel. Linear");

	vLinear.includecategory(&catVLinear[0]);
	vLinear.includecategory(&catVLinear[1]);
	vLinear.includecategory(&catVLinear[2]);
		
	/*Define-se o metodo defuzzificacao: MAXIMUM, AVERAGEOFMAX, or CENTEROFAREA/CENTROID     */
	fuzzyAngularBase.set_defuzz(CENTROID);
	fuzzyLinearBase.set_defuzz(CENTROID);
	
	fuzzyAngularBase.definevars(erroAngular, erroLinear, vAngular);
	fuzzyLinearBase.definevars(erroAngular, erroLinear, vLinear);
	
	//REGRAS ANGULAR
	//com erro negativo, vire pra esquerda
	fuzzyAngularBase.insert_rule ("AltoNeg", "Chegou", "ForteEsq");
	fuzzyAngularBase.insert_rule ("AltoNeg", "Perto", "ForteEsq");
	fuzzyAngularBase.insert_rule ("AltoNeg", "Longe", "ForteEsq");
	fuzzyAngularBase.insert_rule ("BaixoNeg", "Chegou", "Esq");
	fuzzyAngularBase.insert_rule ("BaixoNeg", "Perto", "Esq");
	fuzzyAngularBase.insert_rule ("BaixoNeg", "Longe", "Esq");

	//com erro perto de 0, não mexa
	fuzzyAngularBase.insert_rule ("Reto", "Chegou", "Reto");
	fuzzyAngularBase.insert_rule ("Reto", "Perto", "Reto");
	fuzzyAngularBase.insert_rule ("Reto", "Longe", "Reto");

	//com erro positivo, vire pra direita
	fuzzyAngularBase.insert_rule ("BaixoPos", "Chegou", "Dir");
	fuzzyAngularBase.insert_rule ("BaixoPos", "Perto", "Dir");
	fuzzyAngularBase.insert_rule ("BaixoPos", "Longe", "Dir");
	fuzzyAngularBase.insert_rule ("AltoPos", "Chegou", "ForteDir");
	fuzzyAngularBase.insert_rule ("AltoPos", "Perto", "ForteDir");
	fuzzyAngularBase.insert_rule ("AltoPos", "Longe", "ForteDir");
	
	//REGRAS LINEAR
	//se está longe
		//e está com erro angular alto, "parado".
		fuzzyLinearBase.insert_rule("AltoNeg", "Longe", "Parado");
		fuzzyLinearBase.insert_rule("AltoPos", "Longe", "Parado");
		//senão, acelera
		fuzzyLinearBase.insert_rule("BaixoNeg", "Longe", "Medio");
		fuzzyLinearBase.insert_rule("Reto", "Longe", "Rapido");
		fuzzyLinearBase.insert_rule("BaixoPos", "Longe", "Medio");
	
	//Se está perto, vai em vel. media
	fuzzyLinearBase.insert_rule("AltoNeg", "Perto", "Parado");
	fuzzyLinearBase.insert_rule("BaixoNeg", "Perto", "Medio");
	fuzzyLinearBase.insert_rule("Reto", "Perto", "Rapido");
	fuzzyLinearBase.insert_rule("BaixoPos", "Perto", "Medio");
	fuzzyLinearBase.insert_rule("AltoPos", "Perto", "Parado");
	
	//praticamente chegou, "parado".
	fuzzyLinearBase.insert_rule("AltoNeg", "Chegou", "Parado");
	fuzzyLinearBase.insert_rule("BaixoNeg", "Chegou", "Parado");
	fuzzyLinearBase.insert_rule("Reto", "Chegou", "Parado");
	fuzzyLinearBase.insert_rule("BaixoPos", "Chegou", "Parado");
	fuzzyLinearBase.insert_rule("AltoPos", "Chegou", "Parado");
}

//método que recebe a posição do youbot e extrai o yaw
void subCallback(const geometry_msgs::PoseStamped msg)
{
	//posição x,y,z
	feedback.x = msg.pose.position.x;
	feedback.y = msg.pose.position.y;
	feedback.z = msg.pose.position.z;

	//Extrai os ângulos do quaternion
	tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
	                 msg.pose.orientation.z, msg.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

}

//método que recebe a posição X da ponta do braço
void subCallbackX(const std_msgs::Float32 msg)
{
	pos_x = msg.data;
}

//método que recebe a posição Y da ponta do braço
void subCallbackY(const std_msgs::Float32 msg)
{
	pos_y = msg.data;
}

//método que recebe a posição Z da ponta do braço
void subCallbackZ(const std_msgs::Float32 msg)
{
	pos_z = msg.data;
}

//método que recebe a posição X do cubo na imagem
void subCallbackCuboX(const std_msgs::Float32 msg)
{
	cubo_x = msg.data;
}

//método que recebe a posição Y do cubo na imagem
void subCallbackCuboY(const std_msgs::Float32 msg)
{
	cubo_y = msg.data;
}

//computa a saída que será mandada na dimensão linear
void computaMsgLinear() {
	
	//Computa erro linear, através da fórmula da hipotenusa do triângulo retângulo
	double erro = sqrt(pow(pos_desejada_youbot[0] - feedback.x, 2) + pow(pos_desejada_youbot[1] - feedback.y, 2));

	//computa a saída FUZZY
	//a saída é multiplicada por -1, pois o robô anda "pra frente" com valores negativos.
	lSaida = -1 * fuzzyLinearBase.make_inference(aUltimoErro, erro);
	ROS_INFO("L: Erro>>%lf,	Fuzzy>> %lf", erro, lSaida);

	//guarda valor do erro para computar a mensagem angular
	lUltimoErro = erro;
}

//computa a saída que será mandada na dimensão angular
void computaMsgAngular() {
	
	//Computa erro angular, usando atan2 sobre o ponto desejado em relação ao robô
	double angulo = atan2(pos_desejada_youbot[1] - feedback.y, pos_desejada_youbot[0] - feedback.x);
		
	//caso o yaw venha maior ou menor que PI, corrige.
	if (yaw >= M_PI) yaw = yaw - 2 * M_PI;
	if (yaw <= -M_PI) yaw = yaw + 2 * M_PI;

	//o cálculo do erro nos 4 quadrantes respeita a seguinte fórmula:
	double erro = angulo - yaw;
	//correção caso o erro seja maior ou menor que PI.
	if (erro >= M_PI) erro = erro - 2 * M_PI;
	if (erro <= -M_PI) erro = erro + 2 * M_PI;

	//computa a saída FUZZY
	aSaida = fuzzyAngularBase.make_inference(erro, lUltimoErro);
	ROS_INFO("A: Erro>>%lf, Fuzzy>>%lf", erro, aSaida);

	//guarda valor do erro para computar a mensagem linear
	aUltimoErro = erro;
}

//gira o robô para a orientação correta, uma vez que atingiu a mesa
void calculaMsgYawEmFrenteDaMesa() {
	
	//caso o yaw venha maior ou menor que PI, corrige.
	if (yaw >= M_PI) yaw = yaw - 2 * M_PI;
	if (yaw <= -M_PI) yaw = yaw + 2 * M_PI;

	//o cálculo do erro nos 4 quadrantes respeita a seguinte fórmula:
	double erro = yaw_em_frente_da_mesa - yaw;
	//correção caso o erro seja maior ou menor que PI.
	if (erro >= M_PI) erro = erro - 2 * M_PI;
	if (erro <= -M_PI) erro = erro + 2 * M_PI;

	//computa a saída FUZZY
	aSaida = fuzzyAngularBase.make_inference(erro, lUltimoErro);
	ROS_INFO("A: Yaw Desejado>>%lf,	Yaw>>%lf, Erro>>%lf", yaw_em_frente_da_mesa, yaw, erro);

	//guarda valor do erro para computar a mensagem linear
	aUltimoErro = erro;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrep_youbot");

	ros::NodeHandle n;
	// publisher /cmd_vel, velocidade linear e angular do robô
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	// publisher /is_modo_fk, variável de controle que diz qual modo de controle do braço deve ser
	// 		ativado no momento: (0 = nada, 1 = fk, 2 = ik)
	ros::Publisher pubIsModoFk = n.advertise<std_msgs::Int32>("/is_modo_fk", 1000);
	//  publishers para os valores das juntas do robô, aplicadas no mofo FK 
	ros::Publisher pubJoint1 = n.advertise<std_msgs::Float32>("/joint_1", 1000);
	ros::Publisher pubJoint2 = n.advertise<std_msgs::Float32>("/joint_2", 1000);
	ros::Publisher pubJoint3 = n.advertise<std_msgs::Float32>("/joint_3", 1000);
	ros::Publisher pubJoint4 = n.advertise<std_msgs::Float32>("/joint_4", 1000);
	ros::Publisher pubJoint5 = n.advertise<std_msgs::Float32>("/joint_5", 1000);
	// publishers para o valor do ponto de atuação desejado x,y,z, aplicado no modo IK
	ros::Publisher pubIkX = n.advertise<std_msgs::Float32>("/ik_desired_x", 1000);
	ros::Publisher pubIkY = n.advertise<std_msgs::Float32>("/ik_desired_y", 1000);
	ros::Publisher pubIkZ= n.advertise<std_msgs::Float32>("/ik_desired_z", 1000);
	// publisher do estado da garra (0 = aberta, 1 = fechada)
	ros::Publisher pubIsGarraFechada = n.advertise<std_msgs::Int32>("/is_garra_fechada", 1000);
	
	// sub odometria
	ros::Subscriber sub = n.subscribe("/odom", 1000, subCallback);
	// sub posicao da ponta do braço
	ros::Subscriber subTipX = n.subscribe("/tip_x", 1000, subCallbackX);
	ros::Subscriber subTipY = n.subscribe("/tip_y", 1000, subCallbackY);
	ros::Subscriber subTipZ = n.subscribe("/tip_z", 1000, subCallbackZ);
	// sub posicao do cubo na imagem
	ros::Subscriber subCuboX = n.subscribe("/cubo_x", 1000, subCallbackCuboX);
	ros::Subscriber subCuboY = n.subscribe("/cubo_y", 1000, subCallbackCuboY);

	ros::Rate loop_rate(10);

	if (ros::ok())
	{
		//inicializando variáveis para a primeira execução
		geometry_msgs::Twist msg;
		float aTolerance = 0.05, lTolerance = 0.05;
		
		//loga a execução etapa a etapa em um arquivo csv, durante a execução
		std::ofstream csv;
      	csv.open ("log.csv");
      	csv << "erro_linear,erro_angular,saida_linear,saida_angular,ros_time,\n";

		//prepara os controladores fuzzy
		preparaControleFuzzyBase();

		//mensagens para posição do braço
		std_msgs::Int32 msg_int;
		std_msgs::Float32 msg_float[5];
		//mensagens para ik desejado
		std_msgs::Float32 msg_ik_desejado[3];

		//para leitura do destino a partir do teclado
		// cout << "Digite a posicao\nX>>";
		// cin >> pos_desejada_youbot[0];

		// cout << "Y>>";
		// cin >> pos_desejada_youbot[1];

		/* Posição desejada: em frente à mesa */
		pos_desejada_youbot[0] = pos_em_frente_da_mesa[0];
		pos_desejada_youbot[1] = pos_em_frente_da_mesa[1];

		// primeira coisa é setar o modo para 0 (0 = nada, 1 = fk, 2 = ik)
		msg_int.data = 0;
		pubIsModoFk.publish(msg_int);
		//setar também as coordenadas ik desejadas para 0
		msg_ik_desejado[0].data = 0;
		msg_ik_desejado[1].data = 0;
		msg_ik_desejado[2].data = 0;
		pubIkX.publish(msg_ik_desejado[0]);
		pubIkY.publish(msg_ik_desejado[1]);
		pubIkZ.publish(msg_ik_desejado[2]);

		// primeira coisa é setar o estado da garra para aberta
		msg_int.data = 0;
		pubIsGarraFechada.publish(msg_int);

		ros::spinOnce();
		loop_rate.sleep();

		ROS_INFO("PosicaoYouBot: X>>%lf, Y>>%lf, Yaw>>%lf", feedback.x, feedback.y, yaw);

		//enquanto o objetivo não foi atingido
		while ((ros::ok()) && (abs(lUltimoErro) > lTolerance)) {

			// Controle da orientação
			computaMsgAngular();
			msg.angular.z = aSaida;
			
			// Controle da posição
			computaMsgLinear();
			msg.linear.x = lSaida;
			
			// LOG em arquivo CSV
			csv << lUltimoErro << "," << aUltimoErro << ","
				<< lSaida << "," << aSaida << "," << ros::Time::now() << ",\n";
		    
			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}

		ROS_WARN("...Posicao alcancada...");
		ROS_INFO("PosicaoYouBot: X>>%lf, Y>>%lf, Yaw>>%lf", feedback.x, feedback.y, yaw);
		msg.linear.x = 0;
		lSaida = 0;
		msg.angular.z = 0;
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		/* Uma vez na frente da mesa, gira o robô na orientação correta*/
		while ((ros::ok()) && (abs(yaw - yaw_em_frente_da_mesa) > 0.05)) {

			computaMsgLinear();
			calculaMsgYawEmFrenteDaMesa();

			//correção para que o robô não demore para ajustar, quando bem próximo da orientação final
			if ((aSaida < 0) && (aSaida > -0.1)) {
				aSaida = -0.1;
			} else if ((aSaida > 0) && (aSaida < 0.1)) {
				aSaida = 0.1;
			}

			// LOG em arquivo CSV
			csv << lUltimoErro << "," << aUltimoErro << ","
				<< 0 << "," << aSaida << "," << ros::Time::now() << ",\n";

			msg.angular.z = aSaida;

			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}

		ROS_WARN("...Orientacao alcancada...");
		ROS_INFO("PosicaoYouBot: X>>%lf, Y>>%lf, Yaw>>%lf", feedback.x, feedback.y, yaw);

		//fecha o arquivo csv
		csv.close();
		    
		msg.linear.x = 0;
		msg.angular.z = 0;
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		/* Agora o robô está na posição ideal para buscar o objeto na mesa. */
		/* Envia mensagens para posicionar o braço do robô sobre a mesa, apontado para baixo.*/
		/* Dessa forma, a câmera fica voltada para a mesa para identificar os objetos. */

		// indica se e modo FK ou nao (0 = nada, 1 = fk, 2 = ik)
		msg_int.data = 1;
		pubIsModoFk.publish(msg_int);

		//posição ideal da câmera
		msg_float[0].data = 0;
		msg_float[1].data = -15;
		msg_float[2].data = -50;
		msg_float[3].data = -102;
		msg_float[4].data = 0;

		pubJoint1.publish(msg_float[0]);
		pubJoint2.publish(msg_float[1]);
		pubJoint3.publish(msg_float[2]);
		pubJoint4.publish(msg_float[3]);
		pubJoint5.publish(msg_float[4]);

		ROS_WARN("...Posicionando camera sobre a mesa...");

		ros::spinOnce();
		loop_rate.sleep();

		ros::Duration(20).sleep();
		
		ros::spinOnce();
		loop_rate.sleep();

		// pega posicao x/y do objeto na imagem
		ROS_INFO("Posicao da ponta do braco: X>>%lf, Y>>%lf, Z>>%lf", pos_x, pos_y, pos_z);
		
		//levanta braco com o cubo capturado até ponto intermetiario 2
			//envia x/y/z no modo IK
			//while posicao nao atingida, aguarda.
		//leva braco a posicao final na base
			//envia x/y/z no modo IK
			//while posicao nao atingida, aguarda.
		//solta cubo

		// após isso, ativa modo IK
		// indica se e modo FK ou nao (0 = nada, 1 = fk, 2 = ik)
		msg_int.data = 2;
		pubIsModoFk.publish(msg_int);
		ros::spinOnce();
		loop_rate.sleep();

		ros::Duration(1).sleep();
		ros::spinOnce();

		//pega a posição do cubo
		ROS_INFO("Posicao do cubo na imagem: X>>%lf, Y>>%lf", cubo_x, cubo_y);
		// sabe-se a altura da mesa e do cubo
		// posiciona o braco no lugar correto para pegar o cubo
		// sabendo a altura da mesa, calcula-se a posição x,y,z do cubo.
		// envia x/y/z no modo IK:
		msg_ik_desejado[0].data = 0.610;
		msg_ik_desejado[1].data = 0;
		msg_ik_desejado[2].data = -0.033;

		pubIkX.publish(msg_ik_desejado[0]);
		pubIkY.publish(msg_ik_desejado[1]);
		pubIkZ.publish(msg_ik_desejado[2]);

		ros::spinOnce();		
		loop_rate.sleep();

		// while posicao nao atingida, aguarda.
		ros::Duration(2).sleep();

		// atingiu posicao, pega o cubo
		msg_int.data = 1;
		pubIsGarraFechada.publish(msg_int);
		
		// aguarda um tempo
		ros::spinOnce();		
		loop_rate.sleep();

		ROS_INFO("Pega o cubo.");

		ros::Duration(3).sleep();

		// indica se e modo FK ou nao (0 = nada, 1 = fk, 2 = ik)
		msg_int.data = 1;
		pubIsModoFk.publish(msg_int);

		//posição ideal para largar o cubo na base
		msg_float[0].data = 0;
		msg_float[1].data = 33;
		msg_float[2].data = 62.5;
		msg_float[3].data = 53;
		msg_float[4].data = 0;

		pubJoint1.publish(msg_float[0]);
		pubJoint2.publish(msg_float[1]);
		pubJoint3.publish(msg_float[2]);
		pubJoint4.publish(msg_float[3]);
		pubJoint5.publish(msg_float[4]);

		ROS_INFO("Posiciona braço para largar cubo na base.");

		ros::spinOnce();
		loop_rate.sleep();

		ros::Duration(20).sleep();
		
		ROS_INFO("Larga o cubo.");

		// atingiu posicao, larga o cubo
		msg_int.data = 0;
		pubIsGarraFechada.publish(msg_int);
		
		// aguarda um tempo
		ros::spinOnce();		
		loop_rate.sleep();

		ros::Duration(3).sleep();

	}
	return 0;
}
