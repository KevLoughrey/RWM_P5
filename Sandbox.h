#ifndef SANDBOX_H
#define SANDBOX_H

class Sandbox : public Test
{
public:
	float m_scale;
	b2PrismaticJoint* m_j;
	Sandbox() 
	{

#pragma region Scale
		m_scale = 1 / 4.0f;
#pragma endregion

#pragma region Ground
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 3), b2Vec2(4000.0f, 3));
			ground->CreateFixture(&shape, 0.0f);
		}
#pragma endregion

		{
#pragma region Piston
			b2PolygonShape shape;
			shape.SetAsBox(4.0f * m_scale, 4.5f * m_scale);
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(3.0f, 6.875f);
			b2Body* pistonEnd = m_world->CreateBody(&bd);
			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.groupIndex = -1;
			pistonEnd->CreateFixture(&fd);

			shape.SetAsBox(11.0f * m_scale, 1.0f * m_scale, 15.0f * m_scale, 0.0f * m_scale, 0); //Piston rod - offset to the right of the piston end
			pistonEnd->CreateFixture(&fd);

			shape.SetAsBox(1.5f * m_scale, 7.25f * m_scale, 27.5f * m_scale, 0.0f * m_scale, 0); //Piston block - offset to the right of the piston rod
			pistonEnd->CreateFixture(&fd);

			shape.SetAsBox(5.75f * m_scale, 1.25 * m_scale, 0, 5.75f * m_scale, 0);
			pistonEnd->CreateFixture(&fd);

			shape.SetAsBox(5.75f * m_scale, 1.25 * m_scale, 0, -5.75f * m_scale, 0);
			pistonEnd->CreateFixture(&fd);
#pragma endregion 

#pragma region CrossHeadAxle
			shape.SetAsBox(14.5f * m_scale, 1.25f * m_scale);
			bd.type = b2_dynamicBody;
			bd.position.Set(0, 8.5f);
			b2Body* crossHeadAxle = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.groupIndex = -1;
			crossHeadAxle->CreateFixture(&fd);

			shape.SetAsBox(13.35f * m_scale, 9.4f * m_scale, 27.85f * m_scale, -4.8f * m_scale, 0); //Cylinder offset to the right side of the crossHeadAxle
			b2ChainShape cs;  //Convert to chain shape.
			const b2Vec2 outerVertices[4] = {
				shape.GetVertex(3),
				shape.GetVertex(0),
				shape.GetVertex(1),
				shape.GetVertex(2)};
			cs.CreateChain(outerVertices, 4);
			fd.shape = &cs;
			crossHeadAxle->CreateFixture(&fd);

			cs = b2ChainShape();
			shape.SetAsBox(10.0f * m_scale, 8.85f * m_scale, 27.85f * m_scale, -4.25f * m_scale, 0); //Offset to the left side of the crossHeadAxle
			const b2Vec2 innerVertices[4] = {
				shape.GetVertex(3),
				shape.GetVertex(0),
				shape.GetVertex(1),
				shape.GetVertex(2)};
			cs.CreateChain(innerVertices, 4);
			crossHeadAxle->CreateFixture(&fd);
			
			b2Vec2 vecA(4.4625f, 1.15f);
			b2Vec2 vecB(3.2f, 1.15f);
			b2Vec2 vecC(3.2f, 3.65f);
			b2Vec2 vecD(6, 3.65f);

			cs = b2ChainShape();
			const b2Vec2 topLeftVertices[4] = {
				vecA,
				vecB,
				vecC,
				vecD};
			cs.CreateChain(topLeftVertices, 4);
			crossHeadAxle->CreateFixture(&fd);

			vecA = b2Vec2(9.4625f, 1.15f);
			vecB = b2Vec2(10.7f, 1.15f);
			vecC = b2Vec2(10.7f, 3.65f);
			vecD = b2Vec2(8, 3.65f);

			cs = b2ChainShape();
			const b2Vec2 topRightVertices[4] = {
				vecA,
				vecB,
				vecC,
				vecD};
			cs.CreateChain(topRightVertices, 4);
			crossHeadAxle->CreateFixture(&fd);

			shape.SetAsBox(8.0f * m_scale, 1.5f * m_scale, 27.85f * m_scale, 3.1f * m_scale, 0);
			fd.shape = &shape;
			crossHeadAxle->CreateFixture(&fd);

			shape.SetAsBox(14.5f * m_scale, 1.25f * m_scale, 0, -11.5 * m_scale, 0);
			bd.type = b2_dynamicBody;
			m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.groupIndex = -1;
			crossHeadAxle->CreateFixture(&fd);

			shape.SetAsBox(1.0f * m_scale, 1.5f * m_scale, -15.5f * m_scale, 0, 0); //Offset to the left side of the crossHeadAxle
			crossHeadAxle->CreateFixture(&fd);

			shape.SetAsBox(1.0f * m_scale, 1.5f * m_scale, -15.5f * m_scale, -11.5 * m_scale, 0); //Offset to the left side of the crossHeadAxle
			crossHeadAxle->CreateFixture(&fd);
#pragma endregion

#pragma region Prismatic_Joint_(Piston_and_Cross-Head_Axle)
			b2PrismaticJointDef jd;
			jd.bodyA = pistonEnd;
			jd.bodyB = crossHeadAxle;
			jd.collideConnected = false;
			jd.localAnchorA = b2Vec2(0, 5.75f * m_scale);
			jd.localAxisA = b2Vec2(1, 0);
			//jd.upperTranslation = 9 * m_scale;
			//jd.lowerTranslation = -9 * m_scale;
			//jd.enableLimit = true;
			m_j = (b2PrismaticJoint*)m_world->CreateJoint(&jd);
#pragma endregion

#pragma region MainRod
			shape.SetAsBox(23.0f * m_scale, 1.5f * m_scale);
			bd.type = b2_dynamicBody;
			bd.position.Set(-9.0f, 7.2f);
			b2Body* mainRod = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			mainRod->CreateFixture(&fd);

			shape.SetAsBox(3.5f * m_scale, 2.5f * m_scale, 26.5f * m_scale, 0, 0); //Piston rod - offset to the right of the piston end
			mainRod->CreateFixture(&fd);

			shape.SetAsBox(3.5f * m_scale, 2.5f * m_scale, -26.5f * m_scale, 0, 0); //Piston rod - offset to the right of the piston end
			mainRod->CreateFixture(&fd);
#pragma endregion

#pragma region Revolute_Joint_(Piston_and_Main_Rod)
			b2RevoluteJointDef rjd;
			rjd.bodyA = pistonEnd;
			rjd.bodyB = mainRod;
			rjd.localAnchorB = b2Vec2(26.5f * m_scale, -0.5f * m_scale);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion

#pragma region WheelOne
			b2CircleShape cshape;
			cshape.m_radius = 17 * m_scale;
			bd.type = b2_dynamicBody;
			bd.position.Set(-13.25f, 7.2f);
			b2Body* wheelOne = m_world->CreateBody(&bd);
			fd.shape = &cshape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			wheelOne->CreateFixture(&fd);
#pragma endregion

#pragma region WheelSpinner
			cshape.m_radius = 1 * m_scale;
			bd.type = b2_dynamicBody;
			bd.position.Set(-13.25f, 9.0f);
			b2Body* wheelSpinner = m_world->CreateBody(&bd);
			fd.shape = &cshape;
			fd.density = 30.0f;
			fd.filter.categoryBits = -1;
			wheelSpinner->CreateFixture(&fd);
#pragma endregion

#pragma region W_Joint_(WheelOne_and_Spinner)
			b2WeldJointDef wjd;
			wjd.bodyA = wheelOne;
			wjd.bodyB = wheelSpinner;
			wjd.localAnchorB = b2Vec2(0, -8 * m_scale);
			wjd.collideConnected = false;
			m_world->CreateJoint(&wjd);
#pragma endregion
			
#pragma region Revolute_Joint_(WheelOne_and_Main_Rod)
			rjd.bodyA = wheelSpinner;
			rjd.bodyB = mainRod;
			rjd.localAnchorB = b2Vec2(-26.5f * m_scale, 0);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion

#pragma region WheelTwo
			cshape.m_radius = 17 * m_scale;
			bd.type = b2_dynamicBody;
			bd.position.Set(-4.25f, 7.2f);
			b2Body* wheelTwo = m_world->CreateBody(&bd);
			fd.density = 1.0f;
			wheelTwo->CreateFixture(&fd);
#pragma endregion

#pragma region WheelTwoSpinner
			cshape.m_radius = 1 * m_scale;
			bd.type = b2_dynamicBody;
			bd.position.Set(-4.25f, 9.0f);
			b2Body* wheelTwoSpinner = m_world->CreateBody(&bd);
			fd.density = 30.0f;
			wheelTwoSpinner->CreateFixture(&fd);
#pragma endregion
			
#pragma region W_Joint_(WheelTwo_and_Spinner)
			wjd.bodyA = wheelTwo;
			wjd.bodyB = wheelTwoSpinner;
			wjd.localAnchorB = b2Vec2(0, -8 * m_scale);
			wjd.collideConnected = false;
			m_world->CreateJoint(&wjd);
#pragma endregion

#pragma region WheelRod
			shape.SetAsBox(16.0f * m_scale, 1.5f * m_scale);
			bd.type = b2_dynamicBody;
			bd.position.Set(-8.75f, 9.0f);

			b2Body* wheelRod = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			wheelRod->CreateFixture(&fd);

			shape.SetAsBox(2.0f * m_scale, 2.0f * m_scale, 18.0f * m_scale, 0, 0);
			wheelRod->CreateFixture(&fd);

			shape.SetAsBox(2.0f * m_scale, 2.0f * m_scale, -18.0f * m_scale, 0, 0);
			wheelRod->CreateFixture(&fd);
#pragma endregion

#pragma region Revolute_Joint_(WheelOne_and_Wheel_Rod)
			rjd.bodyA = wheelSpinner;
			rjd.bodyB = wheelRod;
			rjd.localAnchorB = b2Vec2(-18.0f * m_scale, 0);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion
			
#pragma region Revolute_Joint_(WheelTwo_and_Wheel_Rod)
			rjd.bodyA = wheelTwoSpinner;
			rjd.bodyB = wheelRod;
			rjd.localAnchorB = b2Vec2(18.0f * m_scale, 0);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion

#pragma region WheelThree
			cshape.m_radius = 17 * m_scale;
			bd.type = b2_dynamicBody;
			bd.position.Set(-22.25f, 7.2f);
			b2Body* wheelThree = m_world->CreateBody(&bd);
			fd.shape = &cshape;
			fd.density = 1.0f;
			wheelThree->CreateFixture(&fd);
#pragma endregion

#pragma region WheelThreeSpinner
			cshape.m_radius = 1 * m_scale;
			bd.type = b2_dynamicBody;
			bd.position.Set(-22.25f, 9.0f);
			b2Body* wheelThreeSpinner = m_world->CreateBody(&bd);
			fd.density = 30.0f;
			wheelThreeSpinner->CreateFixture(&fd);
#pragma endregion
			
#pragma region W_Joint_(WheelThree_and_Spinner)
			wjd.bodyA = wheelThree;
			wjd.bodyB = wheelThreeSpinner;
			wjd.localAnchorB = b2Vec2(0, -8 * m_scale);
			wjd.collideConnected = false;
			m_world->CreateJoint(&wjd);
#pragma endregion

#pragma region WheelRodTwo
			shape.SetAsBox(16.0f * m_scale, 1.5f * m_scale);
			bd.type = b2_dynamicBody;
			bd.position.Set(-12.75f, 9.0f);
			b2Body* wheelRodTwo = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			wheelRodTwo->CreateFixture(&fd);

			shape.SetAsBox(2.0f * m_scale, 2.0f * m_scale, 18.0f * m_scale, 0, 0);
			wheelRodTwo->CreateFixture(&fd);

			shape.SetAsBox(2.0f * m_scale, 2.0f * m_scale, -18.0f * m_scale, 0, 0);
			wheelRodTwo->CreateFixture(&fd);
#pragma endregion

#pragma region Carriage
			shape.SetAsBox(120.0f * m_scale, 40.0f * m_scale);
			bd.type = b2_kinematicBody;
			bd.position.Set(-8.0f, 17.0f);
			bd.gravityScale = 0;
			bd.fixedRotation = true;
			b2Body* carriage = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			carriage->CreateFixture(&fd);
#pragma endregion

#pragma region Revolute_Joint_(WheelOne_and_Wheel_Rod_Two)
			rjd.bodyA = wheelSpinner;
			rjd.bodyB = wheelRodTwo;
			rjd.localAnchorB = b2Vec2(18.0f * m_scale, 0);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion

#pragma region Revolute_Joint_(WheelThree_and_Wheel_Rod_Two)
			rjd.bodyA = wheelThreeSpinner;
			rjd.bodyB = wheelRodTwo;
			rjd.localAnchorB = b2Vec2(-18.0f * m_scale, 0);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion
			
#pragma region R_Joint_(WheelandCarriage)
			rjd.bodyA = wheelOne;
			rjd.bodyB = carriage;
			rjd.localAnchorB = b2Vec2(-5.15f, -9.8f);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion
			
#pragma region R_Joint_(Wheel2andCarriage)
			rjd.bodyA = wheelTwo;
			rjd.bodyB = carriage;
			rjd.localAnchorB = b2Vec2(3.9f, -9.8f);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion			

#pragma region R_Joint_(Wheel3andCarriage)
			rjd.bodyA = wheelThree;
			rjd.bodyB = carriage;
			rjd.localAnchorB = b2Vec2(-14.15f, -9.8f);
			rjd.collideConnected = false;
			m_world->CreateJoint(&rjd);
#pragma endregion

#pragma region W_Joint_(CarriageAndAxle)
			wjd.bodyA = crossHeadAxle;
			wjd.bodyB = carriage;
			wjd.localAnchorB = b2Vec2(32.125f * m_scale, -32.5f * m_scale);
			wjd.collideConnected = false;
			m_world->CreateJoint(&wjd);
#pragma endregion

#pragma region Thing
			shape.SetAsBox(9.0f * m_scale, 2.0f * m_scale);
			bd.type = b2_dynamicBody;
			bd.gravityScale = 1;
			bd.fixedRotation = false;
			bd.position.Set(-7.0f, 11.0f);
			b2Body* thing = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			thing->CreateFixture(&fd);
#pragma endregion

#pragma region WJoint_Thing_Carriage
			wjd.bodyA = thing;
			wjd.bodyB = carriage;
			wjd.localAnchorB = b2Vec2(1.0f, -6.0f);
			m_world->CreateJoint(&wjd);
#pragma endregion

#pragma region ThingTwo
			shape.SetAsBox(1.5f * m_scale, 10.0f * m_scale);
			bd.position.Set(-5.125f, 11.0f);
			b2Body* thingTwo = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 0.01f;
			fd.filter.categoryBits = -1;
			thingTwo->CreateFixture(&fd);
#pragma endregion

#pragma region RJoint_Thing_Thing2
			rjd.bodyA = thing;
			rjd.bodyB = thingTwo;
			rjd.localAnchorA = b2Vec2(1.875f, 0.5f);
			rjd.localAnchorB = b2Vec2(0, 0.5f);
			//rjd.referenceAngle = thingTwo->GetAngle();
			//rjd.upperAngle = 0.2f;
			//rjd.lowerAngle = -0.2f;
			//rjd.enableLimit = true;
			m_world->CreateJoint(&rjd);
#pragma endregion

#pragma region ThingTwo
			shape.SetAsBox(1.5f * m_scale, 3.5f * m_scale, -0.50f, -2.9f, -0.6545f);
			fd.shape = &shape;
			fd.density = 0.10f;
			fd.filter.categoryBits = -1;
			thingTwo->CreateFixture(&fd);
#pragma endregion
		
#pragma region WheelThing
			shape.SetAsBox(2.5f * m_scale, 7.0f * m_scale);
			bd.position.Set(-13.25f, 9.0f);
			b2Body* wheelThing = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 0.1f;
			fd.filter.categoryBits = -1;
			wheelThing->CreateFixture(&fd);
#pragma endregion

#pragma region RJoint_WheelThing_Spinner
			rjd.referenceAngle = 0;
			rjd.bodyA = wheelRod;
			rjd.bodyB = wheelThing;
			rjd.localAnchorA = b2Vec2(-4.5, -0.0f);
			rjd.localAnchorB = b2Vec2(0, 2);
			rjd.enableLimit = false;
			m_world->CreateJoint(&rjd);
#pragma endregion
			
#pragma region RJoint_WheelThing_Wheel
			rjd.bodyA = wheelOne;
			rjd.bodyB = wheelThing;
			rjd.localAnchorA = b2Vec2(-2, 0.0f);
			rjd.localAnchorB = b2Vec2(0, -2);
			m_world->CreateJoint(&rjd);
#pragma endregion
			
#pragma region ConnectorThing
			shape.SetAsBox(15.0f * m_scale, 1.0f * m_scale);
			bd.position.Set(-13.125f, 6.0f);
			b2Body* connectorThing = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			connectorThing->CreateFixture(&fd);
#pragma endregion
			
#pragma region RJoint_ConnectorThing_ThingTwo
			rjd.bodyA = thingTwo;
			rjd.bodyB = connectorThing;
			rjd.localAnchorA = b2Vec2(-0.7f, -3.25f);
			rjd.localAnchorB = b2Vec2(4.15f, 0);
			m_world->CreateJoint(&rjd);
#pragma endregion
			
#pragma region RJoint_ConnectorThing_WheelThing
			rjd.bodyA = wheelThing;
			rjd.bodyB = connectorThing;
			rjd.localAnchorA = b2Vec2(0.0f, -1.25f);
			rjd.localAnchorB = b2Vec2(-3.8f, 0);
			m_world->CreateJoint(&rjd);
#pragma endregion
			/*
#pragma region Valve
			shape.SetAsBox(8.125f * m_scale, 2.0f * m_scale);
			bd.type = b2_dynamicBody;
			bd.gravityScale = 0;
			bd.fixedRotation = true;
			bd.position.Set(7.0f, 11.275f);
			b2Body* valve = m_world->CreateBody(&bd);
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.filter.categoryBits = -1;
			valve->CreateFixture(&fd);

			shape.SetAsBox(1.0f * m_scale, 4.9f * m_scale, -2.28, 0, 0);
			valve->CreateFixture(&fd);
			shape.SetAsBox(1.0f * m_scale, 4.9f * m_scale, 2.28, 0, 0);
			valve->CreateFixture(&fd);
			shape.SetAsBox(19.5f * m_scale, 1.0f * m_scale, -7.405, 0.8f, 0);
			valve->CreateFixture(&fd);
#pragma endregion*/

			carriage->SetLinearVelocity(b2Vec2(10, 0));
		}

	}

	void Step(Settings* settings)
	{
		//run the default physics and rendering
		Test::Step(settings); 
	}

	static Test* Create()
	{
		return new Sandbox;
	}
};

#endif