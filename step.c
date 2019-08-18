//fuses sur E2 D9 FF (std horloge interne 8mhz, programm DL via SPI
//fuses sur E2 59 FF pour activer PC6 et interdire le reset
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define MYUBRR 26 //8000000/16/BAUD -1 //F_CPU // ie 52 for 9600

// Pour le moteur pas à pas
volatile int lEtape = 0; //numero de cycle des pas moteur
volatile int sens = 0;
volatile long lesPas = 0;

//messages reçus
volatile int numChar = 0;
volatile char message [12];
volatile int indicateur = 0; // vaut 1 quand dernier caractère reçu ; 0 quand traité

//les Mux
volatile uint8_t leMux0 = 0;
volatile uint8_t leMux1 = 0;


void Moteur(int);
void Envoyer (char*);

int main(void)
{
	char *traiteMessage; //string de traitement du message
	int etapePrec = 7;
	int temp = 0; // conversion de la vitesse reçue

	//broches moteur pas à pas : 2 a 5 les moteurs; 6 = enable - sauf que PC6 c'est le pin de reset
	//théoriquement, il est High si les fuses sont pas set
	DDRC |= (1 << PC5) | (1 << PC4) | (1 << PC3) | (1 << PC2)| (1 << PC6);
	//moteur 2 bobines sens horraire quand etape ++ [PC5 = Marron / PC4 = Rouge] et [PC4 = Jaune / PC3 = Bleu]
	// utilisation en demis pas = 1 tour en 96 pas


	//PWM U et V ( OC1A et OC1B)
	DDRB |= (1 << PB1) | (1 << PB2); 			// les broches B1 et B2 en output
	TCCR1A |= (1 << COM1A1) | (1<< COM1B1) ; 	// mode non inverseur
	TCCR1A |= (1 << WGM10); 					// wgm10 et wgm12 => fast pwm 8 bit
	TCCR1B |= (1 << WGM12); 					// wgm10 et wgm12 => fast pwm 8 bit
	TCCR1B |= (1 << CS10); 						// pas de prescaleing

	//sens des moteurs U et V
	DDRB |= (1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5);


	//ports d'écoute en pull up
	DDRB &= ~(1 << PB6) & ~(1 << PB7);		//as input
	PORTB |= 1 << PB6 | 1 << PB7;			//on --> pull up

	DDRD &= ~(1 << PD5) & ~(1 << PD6) & ~(1 << PD7);	//as input
	PORTD |= 1 << PD5 | 1 << PD6 | 1 << PD7 ;			//on --> pull up

	//ports de sortie X Y Z
	DDRD |= (1 << PD4) | (1 << PD3) | (1 << PD2);


	//set baud rate
	UBRR0H = (MYUBRR >> 8);
	UBRR0L = MYUBRR;

	UCSR0B  = (1 << RXEN0) | (1 << TXEN0);	//RX et TX
	UCSR0B |= (1 << RXCIE0);				// Interrupt pour RX

	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); //registre C pour Control ; 1 stop bit (0 sur USBS0 - inutile) et 8bit char (UCSZ01 & 00)


	//timer en mode CTC
	TCCR0A = (1 << WGM01);					// CTC(
	TCCR0B = (1 << CS02) | (1 << CS00); 	//prescaler = 1024 soit 1/7812 ème de seconde

	OCR0A = 0xF9;							// le delais soit des 30ème de seconde pour 255
	TIMSK0 = (1 << OCIE0A);					// Interrupt pour le delais


	//pull up sur ADC
	DDRC &= ~(1 << PC0) & ~(1 << PC1);		//as input
	PORTC |= 1 << PC0 | 1 << PC1;			//on --> pull up

	//set up du ADC sur les ADC0 et ADC1
	ADMUX = (1 << MUX0) | (1 << MUX1);
	ADMUX |= (1 << REFS0); // utilisation de AVcc comme source
	ADMUX |= (1 << ADLAR); // Ajustement à droite pour résolution 8 bits

	ADCSRB = 0; // free running ne pas oublier auto trigger sur ADCSRA (ADATE)

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // prescaler à 64 pour conversion à 8M/64 = 125k (soit >50 et <200)
	ADCSRA |= (1 << ADATE) | (1 << ADEN) | (1 << ADIE) ; // Auto trigger (free running) | Enable ADC | Enable Interrupt
	ADCSRA |= (1 << ADSC); // commence la conversion

	sei();

	while (1)
	{


		if (indicateur == 1){ // message terminé

			switch (message[1]) {
				case 'W':
				case 'w':

					traiteMessage = message + 3; //adresse de message + 3 efface donc les trois premier char

					if (message[2] == 'a' || message[2] == 'A' || message[2] == '+'){
						PORTC |= (1 << PC6); //enable
						sens = 1;
						lesPas = atol(traiteMessage);
					}
					else if (message[2] == 'h' || message[2] == 'H' || message[2] == '-' || message[2] == 'B' || message[2] == 'b'){
						PORTC |= (1 << PC6); //enable
						sens = -1;
						lesPas = atol(traiteMessage);
					}
					else if (message[2] == 'v' || message[2] == 'V'){
						temp = atol(traiteMessage); //changement de vitesse
						if (temp >0 && temp < 256){
							OCR0A = temp;
						}
					}
					else if (message[2] == '0'){
						PORTC &= ~(1 << PC6);
					}
					else if (message[2] == '?'){
						itoa(lesPas, traiteMessage, 10) ;
						Envoyer(traiteMessage);
					}

				break;

				case 'V':
				case 'v':

					traiteMessage = message + 3; //adresse de message + 3 efface donc les trois premier char
					temp = atol(traiteMessage);
					if (temp >=0 && temp < 256){
						OCR1A = temp;
					}

					if (message[2] == 'a' || message[2] == 'A' || message[2] == '+'){
						PORTB = (PORTB | (1 << PB0)) & ~(1 << PB3) ;
					}
					else if (message[2] == 'h' || message[2] == 'H' || message[2] == '-' || message[2] == 'B' || message[2] == 'b'){
						PORTB = (PORTB | (1 << PB3)) & ~(1 << PB0) ;
					}
				break;

				case 'U':
				case 'u':

					traiteMessage = message + 3; //adresse de message + 3 efface donc les trois premier char
					temp = atol(traiteMessage);
					if (temp >=0 && temp < 256){
						OCR1B = temp;
					}

					if (message[2] == 'a' || message[2] == 'A' || message[2] == '+'){
						PORTB = (PORTB | (1 << PB5)) & ~(1 << PB4) ;
					}
					else if (message[2] == 'h' || message[2] == 'H' || message[2] == '-' || message[2] == 'B' || message[2] == 'b'){
						PORTB = (PORTB | (1 << PB4)) & ~(1 << PB5) ;
					}
				break;

				case 'X':
				case 'x':
					//Envoyer(&message[2]);
					if (message[2] == '1' || message[2] == '+'){
						PORTD |= (1 << PD4);
					}
					else if (message[2] == '0' || message[2] == '-'){
						PORTD &= ~(1 << PD4);
					}
				break;

				case 'Y':
				case 'y':
					if (message[2] == '1' || message[2] == '+'){
						PORTD |= (1 << PD3);
					}
					else if (message[2] == '0' || message[2] == '-'){
						PORTD &= ~(1 << PD3);
					}
				break;

				case 'Z':
				case 'z':
					if (message[2] == '1' || message[2] == '+'){
						PORTD |= (1 << PD2);
					}
					else if (message[2] == '0' || message[2] == '-'){
						PORTD &= ~(1 << PD2);
					}
				break;

				case 'B':
				case 'b':
					//lesBoutons = PINB
					if (message[2] == '1' || message[2] == 'A' || message[2] == 'a'){
						itoa(((PINB & (1<<PINB6)) > 0) ? 1 : 0 , traiteMessage, 10) ;
						Envoyer(traiteMessage);
					}
					else if (message[2] == '2' || message[2] == 'B' || message[2] == 'b'){
						itoa(((PINB & (1<<PINB7)) > 0) ? 1 : 0, traiteMessage , 10) ;
						Envoyer(traiteMessage);
					}
					else if (message[2] == '3' || message[2] == 'C' || message[2] == 'c'){
						itoa( ((PIND & (1<<PIND5)) > 0) ? 1 : 0 , traiteMessage, 10) ;
						Envoyer(traiteMessage);
					}
					else if (message[2] == '4' || message[2] == 'D' || message[2] == 'd'){
						itoa(((PIND & (1<<PIND6)) > 0) ? 1 : 0 , traiteMessage, 10) ;
						Envoyer(traiteMessage);
					}
					else if (message[2] == '5' || message[2] == 'E' || message[2] == 'e'){
						itoa(((PIND & (1<<PIND7)) > 0) ? 1 : 0 , traiteMessage, 10) ;
						Envoyer(traiteMessage);
					}
				break;

				case 'A':
				case 'a':
					if (message[2] == '1' || message[2] == 'A' || message[2] == 'a'){
						itoa(leMux1, traiteMessage,10);
						Envoyer(traiteMessage);
					}
					else if (message[2] == '2' || message[2] == 'B' || message[2] == 'b'){
						itoa(leMux0, traiteMessage,10);
						Envoyer(traiteMessage);
					}
				break;

				case '?': //connecté ?
					Envoyer("1");
				break;

				default:
					break;
			}



			indicateur = 0;
		}

		if (etapePrec != lEtape){
			etapePrec = lEtape;
			lesPas--;
			if (lesPas <= 0){
				sens = 0;
				lesPas = 0;
			}
		}

	}

}

void Moteur(int etape)
{
	switch (etape){
		case 0:
			PORTC =  (PORTC | (1 << PC5)) & ~((1 << PC4) | (1 << PC3) | (1 << PC2));
			//&= ~
			break;
		case 1:
			PORTC =  (PORTC | (1 << PC5) | (1 << PC3)) & ~((1 << PC4) | (1 << PC2));
			//&= ~
			break;
		case 2:
			PORTC = (PORTC | (1 << PC3)) & ~((1 << PC2) | (1 << PC4) | (1 << PC5));
			break;
		case 3:
			PORTC = (PORTC | (1 << PC3) | (1 << PC4)) & ~((1 << PC2) | (1 << PC5));
			break;
		case 4:
			PORTC = (PORTC | (1 << PC4)) & ~((1 << PC5) | (1 << PC3) | (1 << PC2));
			break;
		case 5:
			PORTC = (PORTC | (1 << PC4) | (1 << PC2)) & ~((1 << PC5) | (1 << PC3));
			break;
		case 6:
			PORTC = (PORTC | (1 << PC2)) & ~((1 << PC3) | (1 << PC4) | (1 << PC5));
			break;
		case 7:
			PORTC = (PORTC | (1 << PC2)| (1 << PC5)) & ~((1 << PC3) | (1 << PC4));
			break;

	}

}

void Envoyer(char *lEnvoi)
{
	while ( !(UCSR0A & (1 << UDRE0)) ){} //attend que le buffer soit vide
	//UDR0 = '[';

	for (int i=0; lEnvoi[i] != '\0'; i++){
			while ( !(UCSR0A & (1 << UDRE0)) ){} //attend que le buffer soit vide
			UDR0 = lEnvoi[i];
	}
	//caractère de fin
	while ( !(UCSR0A & (1 << UDRE0)) ){} //attend que le buffer soit vide
				UDR0 = 13;
}

ISR (USART_RX_vect)
{
	char recu;

	recu = UDR0;

	if (indicateur == 0){ // pret à recevoir

		if (recu == '('){
			numChar = 0;
		}
		else if (recu == ')'){
			recu = '\0';
			indicateur = 1;
		}

		if (numChar >= 0 || numChar <12){
			message[numChar]=recu;
			numChar++;
		}

	}

	//UDR0 = recu;

}

ISR (TIMER0_COMPA_vect)
{
	lEtape = lEtape + sens;

	if (lEtape > 7) {
		lEtape = 0;
	}
	if (lEtape < 0) {
		lEtape = 7;
	}
	Moteur (lEtape);
}

ISR (ADC_vect)
{
	uint8_t tmp;
	tmp = ADMUX & 0x0F; // les 4 dernier bits sont le numero du pin en cours de lecture
	if (tmp== 0){
		leMux0 = ADCH; //valeur de ADC 0
		ADMUX++; //ajoute 1 au Mux pour couvertir la broche suivante
	}
	else { // on a activé que ADC0 et ADC1
		leMux1 = ADCH; //valeur de ADC1
		ADMUX &= 0xF0; //RAZ des 4 derniers bits pour reset du Mux à ADC0
	}

}
