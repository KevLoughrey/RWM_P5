#ifndef SANDBOX_H
#define SANDBOX_H
 
class Sandbox : public Test
{
public:
        Sandbox()
        {
 
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
                        // Crosshead Arm
                        b2PolygonShape shape;
                        shape.SetAsBox(1.0f, 1.125f);
                        b2BodyDef bd;
                        bd.type = b2_dynamicBody;
                        bd.position.Set(3.0f, 6.875f);
                        b2Body* pistonEnd = m_world->CreateBody(&bd);
                        b2FixtureDef fd;
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.groupIndex = -1;
                        pistonEnd->CreateFixture(&fd);
 
                        // Piston rod
                        shape.SetAsBox(2.75f, 0.25f, 3.75f, 0, 0);
                        pistonEnd->CreateFixture(&fd);
 
                        // Piston head
                        shape.SetAsBox(0.375, 1.8125, 6.875, 0, 0);
                        pistonEnd->CreateFixture(&fd);
 
                        // Crosshead connector
                        shape.SetAsBox(1.4375f, 0.3125f, 0, 1.4375f, 0);
                        pistonEnd->CreateFixture(&fd);
 
                        // Crosshead connector
                        shape.SetAsBox(1.4375f, 0.3125f, 0, -1.4375f, 0);
                        pistonEnd->CreateFixture(&fd);
#pragma endregion
 
#pragma region CrossHeadAxle
                        // Crosshead Axle
                        shape.SetAsBox(3.625f, 0.3125f);
                        bd.type = b2_dynamicBody;
                        bd.position.Set(0, 8.5f);
                        b2Body* crossHeadAxle = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.groupIndex = -1;
                        crossHeadAxle->CreateFixture(&fd);
 
                        // Cylinder
                        shape.SetAsBox(3.3375f, 2.35f, 6.9625f, -1.2f, 0);
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
                        shape.SetAsBox(2.5f, 2.2125f, 6.9625, -1.0625f, 0);
                        const b2Vec2 innerVertices[4] = {
                                shape.GetVertex(3),
                                shape.GetVertex(0),
                                shape.GetVertex(1),
                                shape.GetVertex(2)};
                        cs.CreateChain(innerVertices, 4);
                        crossHeadAxle->CreateFixture(&fd);
                       
                        // Steam box
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
 
                        shape.SetAsBox(2.0f, 0.375f, 6.9625f, 0.775f, 0);
                        fd.shape = &shape;
                        crossHeadAxle->CreateFixture(&fd);
 
                        shape.SetAsBox(3.625f, 0.3125f, 0, -2.875f, 0);
                        bd.type = b2_dynamicBody;
                        m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.groupIndex = -1;
                        crossHeadAxle->CreateFixture(&fd);
 
                        // Axle bumpers
                        shape.SetAsBox(0.25f, 0.375f, -3.875, 0, 0);
                        crossHeadAxle->CreateFixture(&fd);
 
                        shape.SetAsBox(0.25f, 0.375f, -3.875, -2.875f, 0);
                        crossHeadAxle->CreateFixture(&fd);
#pragma endregion
 
#pragma region Prismatic_Joint_(Piston_and_Cross-Head_Axle)
                        // Prismatic Joint between the piston and the crosshead axle
                        b2PrismaticJointDef jd;
                        jd.bodyA = pistonEnd;
                        jd.bodyB = crossHeadAxle;
                        jd.collideConnected = false;
                        jd.localAnchorA = b2Vec2(0, 1.4375f);
                        jd.localAxisA = b2Vec2(1, 0);
                        m_world->CreateJoint(&jd);
#pragma endregion
 
#pragma region MainRod
                        // Main rod
                        shape.SetAsBox(5.75f, 0.375f);
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-9.0f, 7.2f);
                        b2Body* mainRod = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.categoryBits = -1;
                        mainRod->CreateFixture(&fd);
 
                        shape.SetAsBox(0.875f, 0.625f, 6.625f, 0, 0);
                        mainRod->CreateFixture(&fd);
 
                        shape.SetAsBox(0.875f, 0.625f, -6.625f, 0, 0);
                        mainRod->CreateFixture(&fd);
#pragma endregion
 
#pragma region Revolute_Joint_(Piston_and_Main_Rod)
                        // Revolute - piston and main rod
                        b2RevoluteJointDef rjd;
                        rjd.bodyA = pistonEnd;
                        rjd.bodyB = mainRod;
                        rjd.localAnchorB = b2Vec2(6.625f, -0.125f);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
 
#pragma region WheelOne
                        // First wheel
                        b2CircleShape cshape;
                        cshape.m_radius = 4.25f;
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-13.25f, 7.2f);
                        b2Body* wheelOne = m_world->CreateBody(&bd);
                        fd.shape = &cshape;
                        fd.density = 1.0f;
                        fd.filter.categoryBits = -1;
                        wheelOne->CreateFixture(&fd);
#pragma endregion
 
#pragma region WheelSpinner
                        // Attachment point for main rod
                        cshape.m_radius = 0.25f;
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-13.25f, 9.0f);
                        b2Body* wheelSpinner = m_world->CreateBody(&bd);
                        fd.shape = &cshape;
                        fd.density = 30.0f;
                        fd.filter.categoryBits = -1;
                        wheelSpinner->CreateFixture(&fd);
#pragma endregion
 
#pragma region W_Joint_(WheelOne_and_Spinner)
                        // Weld the spinner to the wheel
                        b2WeldJointDef wjd;
                        wjd.bodyA = wheelOne;
                        wjd.bodyB = wheelSpinner;
                        wjd.localAnchorB = b2Vec2(0, -2);
                        wjd.collideConnected = false;
                        m_world->CreateJoint(&wjd);
#pragma endregion
                       
#pragma region Revolute_Joint_(WheelOne_and_Main_Rod)
                        // Attach the main rod to the wheel
                        rjd.bodyA = wheelSpinner;
                        rjd.bodyB = mainRod;
                        rjd.localAnchorB = b2Vec2(-6.625f, 0);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
 
#pragma region WheelTwo
                        // Second wheel
                        cshape.m_radius = 4.25f;
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-4.25f, 7.2f);
                        b2Body* wheelTwo = m_world->CreateBody(&bd);
                        fd.density = 1.0f;
                        wheelTwo->CreateFixture(&fd);
#pragma endregion
 
#pragma region WheelTwoSpinner
                        // Attachment point for rod on wheel two
                        cshape.m_radius = 0.25f;
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-4.25f, 9.0f);
                        b2Body* wheelTwoSpinner = m_world->CreateBody(&bd);
                        fd.density = 30.0f;
                        wheelTwoSpinner->CreateFixture(&fd);
#pragma endregion
                       
#pragma region W_Joint_(WheelTwo_and_Spinner)
                        // Weld spinner to wheel
                        wjd.bodyA = wheelTwo;
                        wjd.bodyB = wheelTwoSpinner;
                        wjd.localAnchorB = b2Vec2(0, -2);
                        wjd.collideConnected = false;
                        m_world->CreateJoint(&wjd);
#pragma endregion
 
#pragma region WheelRod
                        // Linker between wheels one and two
                        shape.SetAsBox(4.0f, 0.375f);
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-8.75f, 9.0f);
 
                        b2Body* wheelRod = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.categoryBits = -1;
                        wheelRod->CreateFixture(&fd);
 
                        shape.SetAsBox(0.5f, 0.5f, 4.5f, 0, 0);
                        wheelRod->CreateFixture(&fd);
 
                        shape.SetAsBox(0.5f, 0.5f, -4.5f, 0, 0);
                        wheelRod->CreateFixture(&fd);
#pragma endregion
 
#pragma region Revolute_Joint_(WheelOne_and_Wheel_Rod)
                        // Attach rod to wheel one
                        rjd.bodyA = wheelSpinner;
                        rjd.bodyB = wheelRod;
                        rjd.localAnchorB = b2Vec2(-4.5f, 0);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
                       
#pragma region Revolute_Joint_(WheelTwo_and_Wheel_Rod)
                        // Attach rod to wheel two
                        rjd.bodyA = wheelTwoSpinner;
                        rjd.bodyB = wheelRod;
                        rjd.localAnchorB = b2Vec2(4.5f, 0);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
 
#pragma region WheelThree
                        // Wheel three
                        cshape.m_radius = 4.25f;
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-22.25f, 7.2f);
                        b2Body* wheelThree = m_world->CreateBody(&bd);
                        fd.shape = &cshape;
                        fd.density = 1.0f;
                        wheelThree->CreateFixture(&fd);
#pragma endregion
 
#pragma region WheelThreeSpinner
                        // Attachment point on wheel three
                        cshape.m_radius = 0.25f;
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-22.25f, 9.0f);
                        b2Body* wheelThreeSpinner = m_world->CreateBody(&bd);
                        fd.density = 30.0f;
                        wheelThreeSpinner->CreateFixture(&fd);
#pragma endregion
                       
#pragma region W_Joint_(WheelThree_and_Spinner)
                        // Weld the attach point to the wheel
                        wjd.bodyA = wheelThree;
                        wjd.bodyB = wheelThreeSpinner;
                        wjd.localAnchorB = b2Vec2(0, -2);
                        wjd.collideConnected = false;
                        m_world->CreateJoint(&wjd);
#pragma endregion
 
#pragma region WheelRodTwo
                        // Linker between wheels one and three
                        shape.SetAsBox(4.0f, 0.375f);
                        bd.type = b2_dynamicBody;
                        bd.position.Set(-12.75f, 9.0f);
                        b2Body* wheelRodTwo = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.categoryBits = -1;
                        wheelRodTwo->CreateFixture(&fd);
 
                        shape.SetAsBox(0.5f, 0.5f, 4.25f, 0, 0);
                        wheelRodTwo->CreateFixture(&fd);
 
                        shape.SetAsBox(0.5f, 0.5f, -4.25f, 0, 0);
                        wheelRodTwo->CreateFixture(&fd);
#pragma endregion
 
#pragma region Carriage
                        // The carriage
                        shape.SetAsBox(30, 10);
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
                        // Attach rod to wheel one
                        rjd.bodyA = wheelSpinner;
                        rjd.bodyB = wheelRodTwo;
                        rjd.localAnchorB = b2Vec2(4.5f, 0);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
 
#pragma region Revolute_Joint_(WheelThree_and_Wheel_Rod_Two)
                        // Attach rod to wheel three
                        rjd.bodyA = wheelThreeSpinner;
                        rjd.bodyB = wheelRodTwo;
                        rjd.localAnchorB = b2Vec2(-4.5f, 0);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
                       
#pragma region R_Joint_(WheelandCarriage)
                        // Attach wheel one to carriage
                        rjd.bodyA = wheelOne;
                        rjd.bodyB = carriage;
                        rjd.localAnchorB = b2Vec2(-5.15f, -9.8f);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
                       
#pragma region R_Joint_(Wheel2andCarriage)
                        // Attach wheel two to carriage
                        rjd.bodyA = wheelTwo;
                        rjd.bodyB = carriage;
                        rjd.localAnchorB = b2Vec2(3.9f, -9.8f);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion                      
 
#pragma region R_Joint_(Wheel3andCarriage)
                        // Attach wheel three to the carriage
                        rjd.bodyA = wheelThree;
                        rjd.bodyB = carriage;
                        rjd.localAnchorB = b2Vec2(-14.15f, -9.8f);
                        rjd.collideConnected = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
 
#pragma region W_Joint_(CarriageAndAxle)
                        // Weld the axle to the carriage
                        wjd.bodyA = crossHeadAxle;
                        wjd.bodyB = carriage;
                        wjd.localAnchorB = b2Vec2(8.03125f, -8.125f);
                        wjd.collideConnected = false;
                        m_world->CreateJoint(&wjd);
#pragma endregion
 
#pragma region LiftingLink
                        shape.SetAsBox(2.25f, 0.5f);
                        bd.type = b2_dynamicBody;
                        bd.gravityScale = 1;
                        bd.fixedRotation = false;
                        bd.position.Set(-7.0f, 11.0f);
                        b2Body* liftingLink = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.categoryBits = -1;
                        liftingLink->CreateFixture(&fd);
#pragma endregion
 
#pragma region WJoint_LLink_Carriage
                        wjd.bodyA = liftingLink;
                        wjd.bodyB = carriage;
                        wjd.localAnchorB = b2Vec2(1.0f, -6.0f);
                        m_world->CreateJoint(&wjd);
#pragma endregion
 
#pragma region ExpansionLink
                        shape.SetAsBox(0.375f, 2.5f);
                        bd.position.Set(-5.125f, 11.0f);
                        b2Body* expansionLink = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 0.01f;
                        fd.filter.categoryBits = -1;
                        expansionLink->CreateFixture(&fd);
#pragma endregion
 
#pragma region RJoint_LLink_ExpansionLink
                        rjd.bodyA = liftingLink;
                        rjd.bodyB = expansionLink;
                        rjd.localAnchorA = b2Vec2(1.875f, 0.5f);
                        rjd.localAnchorB = b2Vec2(0, 0.5f);
                        m_world->CreateJoint(&rjd);
#pragma endregion
 
#pragma region ExpansionLink
                        shape.SetAsBox(0.375f, 0.875f, -0.50f, -2.9f, -0.6545f);
                        fd.shape = &shape;
                        fd.density = 0.10f;
                        fd.filter.categoryBits = -1;
                        expansionLink->CreateFixture(&fd);
#pragma endregion
               
#pragma region EccentricCrank
                        shape.SetAsBox(0.625f, 1.75f);
                        bd.position.Set(-13.25f, 9.0f);
                        b2Body* eccentricCrank = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 0.1f;
                        fd.filter.categoryBits = -1;
                        eccentricCrank->CreateFixture(&fd);
#pragma endregion
 
#pragma region RJoint_EccentricCrank_Spinner
                        rjd.referenceAngle = 0;
                        rjd.bodyA = wheelRod;
                        rjd.bodyB = eccentricCrank;
                        rjd.localAnchorA = b2Vec2(-4.5, -0.0f);
                        rjd.localAnchorB = b2Vec2(0, 2);
                        rjd.enableLimit = false;
                        m_world->CreateJoint(&rjd);
#pragma endregion
                       
#pragma region RJoint_ECrank_Wheel
                        rjd.bodyA = wheelOne;
                        rjd.bodyB = eccentricCrank;
                        rjd.localAnchorA = b2Vec2(-2, 0.0f);
                        rjd.localAnchorB = b2Vec2(0, -2);
                        m_world->CreateJoint(&rjd);
#pragma endregion
                       
#pragma region EccentricRod
                        shape.SetAsBox(3.75f, 0.25f);
                        bd.position.Set(-13.125f, 6.0f);
                        b2Body* eccentricRod = m_world->CreateBody(&bd);
                        fd.shape = &shape;
                        fd.density = 1.0f;
                        fd.filter.categoryBits = -1;
                        eccentricRod->CreateFixture(&fd);
#pragma endregion
                       
#pragma region RJoint_ERod_ELink
                        rjd.bodyA = expansionLink;
                        rjd.bodyB = eccentricRod;
                        rjd.localAnchorA = b2Vec2(-0.7f, -3.25f);
                        rjd.localAnchorB = b2Vec2(4.15f, 0);
                        m_world->CreateJoint(&rjd);
#pragma endregion
                       
#pragma region RJoint_ECrank_ERod
                        rjd.bodyA = eccentricCrank;
                        rjd.bodyB = eccentricRod;
                        rjd.localAnchorA = b2Vec2(0.0f, -1.25f);
                        rjd.localAnchorB = b2Vec2(-3.8f, 0);
                        m_world->CreateJoint(&rjd);
#pragma endregion
 
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