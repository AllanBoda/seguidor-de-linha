void calibrate() {

	for (int j = 1; j < 3000; j++) {

		motor1.drive(50);
		motor2.drive(-50);

		for (int i = 1; i<6; i++) {
			if (analogRead(i) < minValues[i]) {
				minValues[i] = analogRead(i);
			}
			if (analogRead(i) > maxValues[i]) {
				maxValues[i] = analogRead(i);
			}
		}
	}

	for (int i=0; i<6; i++) {
		threshold[i] = (minValues[i] + maxValues[i]) / 2;
		Serial.print(threshold[i]);
		Serial.print("	");
	}
	Serial.printnl();

	motor1.drive(0);
	motor2.drive(0);
}