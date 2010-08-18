unit UPhysics2D;

{ box2D 2.1.0 translation

  ###  This unit is written based on Box2D maintained by Erin Catto (http://www.box2d.org)
  All type names follow the Delphi custom Txxx and xxx means the corresponding
  type in cpp source. This rule doesn't apply to the following types b2Vec2,
  b2Vec3, b2Mat22 and b2Mat33. Because I think these types are so primary that you
  may want to reuse them in other projects.

  ###  box2D 2.1.0 version is so different from 2.0.1 version that Erin nearly
  overwrote all codes. For more information I recommend reading the following page
  http://box2dflash.boristhebrave.com/docs/2.1a/updating

  ###  I translated version 2.0.1 without considering much of class encapsulation so
  you can find many m_xxx class memebers left out in public field. In this version
  I fixed all things and you should use Set/Get procedures to access these members.
  But for some members owning both Set and Get procedures I changed them to
  properties. For example I simplify SetUserData and GetUserData to such code
     Property UserData: Pointer read m_userData write m_userData;
  Another thing is that I changed some reference declaration(delphi doesn't support)
  to pointer to avoid unnecessary memory copy. You can check Tb2Body.GetTransform
  and Tb2DynamicTree.GetFatAABB for more details.

  ###  Another difference from version 2.0.1 is that I changed Tb2Contact
  declaration from class to record for effciency. Because class is created on
  heap and Tb2Contact is so frequently created and destroyed.

  ###  Because versions before Delphi 2006 don't support operator overloading, so
  I write two versions of all math operations for vector and matrix, etc. But
  later I found that the version without operator overloading runs faster.
  So if you want a better performance, DEFINE BETTER_PERFORMANCE in Physics2D.inc
  which will UNDEFINE OP_OVERLOAD even if you are using Delphi 2010.

  ###  This library supports three kinds of floats, Single(32bit), Double(64bit) and
  Extended(80bit). Double precision is always recommended because I will test all
  codes and demos using double precision. Using other float systems may cause some
  unpredictable exceptions.
       flags        EXTENDED_PRECISION        DOUBLE_PRECISION
     Extended               ON                     whatever
      Double(default)       OFF                       ON
      Single                OFF                       OFF
  There is also a flag SINGLE_PRECISION in the include file but it doesn't affect
  Float type definition.

  ###  Controllers are added as enhancement and can be flagged by CONTROLLERS.
  If you don't need them, please unflag to reduce code size.

  ###  If you want to do benchmark or something else, please flag COMPUTE_PHYSICSTIME.
  Time consumed by each step is updated and stored in Tb2World.GetPhysicsTime.

  ###  All assertions are ignored.

  Translator: Qianyuan Wang(ÍõÇ¬Ôª)
  Contact me: http://hi.baidu.com/wqyfavor
              wqyfavor@163.com
              QQ: 466798985
}

interface
{$I Physics2D.inc}
{$IFDEF D2009UP}
{$POINTERMATH ON}
{$ENDIF}

uses
   {$IFDEF COMPUTE_PHYSICSTIME}Windows,{$ENDIF}
   UPhysics2DTypes,
   Math,
   Classes;

type
   RGBA = array[0..3] of Single;
   TRGBA = packed record
      red, green, blue, alpha: Single;
   end;

   Tb2BodyDef = class;
   Pb2Body = ^Tb2Body;
   Tb2Body = class;
   Tb2JointDef = class;
   Pb2Joint = ^Tb2Joint;
   Tb2Joint = class;
   Pb2Contact = ^Tb2Contact;
   Pb2Fixture = ^Tb2Fixture;
   Tb2Fixture = class;
   Pb2Shape = ^Tb2Shape;
   Tb2Shape = class;
   Tb2BroadPhase = class;
   Tb2ContactFilter = class;
   Tb2ContactListener = class;
   Tb2ContactManager = class;
   Tb2QueryCallback = class;
   Tb2RayCastCallback = class;
   Tb2Island = class;
   {$IFDEF CONTROLLERS}
   Tb2Controller = class;
   {$ENDIF}

   /// The features that intersect to form the contact point
   /// This must be 4 bytes or less.
   Tb2ContactFeature = record
      indexA: UInt8; ///< Feature index on shapeA
      indexB: UInt8; ///< Feature index on shapeB
      typeA: UInt8;  ///< The feature type on shapeA
      typeB: UInt8;  ///< The feature type on shapeB
   end;

   Tb2ContactID = record
      /// The features that intersect to form the contact point
      case Integer of
         0: (cf: Tb2ContactFeature);
         1: (key: UInt32); ///< Used to quickly compare contact ids.
   end;

   /// A distance proxy is used by the GJK algorithm.
   /// It encapsulates any shape.
   Pb2DistanceProxy = ^Tb2DistanceProxy;
   Tb2DistanceProxy = record
      m_vertices: PVector2;
      m_count: Int32;
      m_radius: Float;
      m_buffer: array[0..1] of TVector2;

      {$IFDEF OP_OVERLOAD}
      /// Initialize the proxy using the given shape. The shape
      /// must remain in scope while the proxy is in use.
      procedure SetShape(shape: Tb2Shape; index: Int32);

      /// Get the supporting vertex index in the given direction.
      function GetSupport(const d: TVector2): Int32;

      /// Get the supporting vertex in the given direction.
      function GetSupportVertex(const d: TVector2): PVector2;
      {$ENDIF}
   end;

   /// Input for b2Distance.
   /// You have to option to use the shape radii in the computation. Even
   Tb2DistanceInput = record
      proxyA, proxyB: Tb2DistanceProxy;
      transformA, transformB: Tb2Transform;
      useRadii: Boolean;
   end;

   /// Output for b2Distance.
   Tb2DistanceOutput = record
      pointA,            ///< closest point on shapeA
      pointB: TVector2;  ///< closest point on shapeB
      distance: Float;
      iterations: Int32; ///< number of GJK iterations used
   end;

   /// Used to warm start b2Distance.
   /// Set count to zero on first call.
   Tb2SimplexCache = record
      metric: Float;		///< length or area
      count: UInt16;
      indexA,
      indexB: array[0..2] of UInt8; /// vertices on shape A an B
   end;

   /// Input parameters for b2TimeOfImpact
   Tb2TOIInput = record
      proxyA, proxyB: Tb2DistanceProxy;
      sweepA, sweepB: Tb2Sweep;
      tMax: Float; // defines sweep interval [0, tMax]
   end;

   // Output parameters for b2TimeOfImpact.
   Tb2TOIOutputState = (e_toi_unknown, e_toi_failed, e_toi_overlapped,
      e_toi_touching, e_toi_separated);
   Tb2TOIOutput = record
      state: Tb2TOIOutputState;
      t: Float;
   end;

   /// A manifold point is a contact point belonging to a contact
   /// manifold. It holds details related to the geometry and dynamics
   /// of the contact points.
   /// The local point usage depends on the manifold type:
   /// -e_circles: the local center of circleB
   /// -e_faceA: the local center of cirlceB or the clip point of polygonB
   /// -e_faceB: the clip point of polygonA
   /// This structure is stored across time steps, so we keep it small.
   /// Note: the impulses are used for internal caching and may not
   /// provide reliable contact forces, especially for high speed collisions.     
   Pb2ManifoldPoint = ^Tb2ManifoldPoint;
   Tb2ManifoldPoint = record
      localPoint: TVector2;		///< usage depends on manifold type    
      normalImpulse: Float; ///< the non-penetration impulse
      tangentImpulse: Float; ///< the friction impulse
      id: Tb2ContactID; ///< uniquely identifies a contact point between two shapes
      isNew: Boolean;
   end;

   /// A manifold for two touching convex shapes.
   /// Box2D supports multiple types of contact:
   /// - clip point versus plane with radius
   /// - point versus point with radius (circles)
   /// The local point usage depends on the manifold type:
   /// -e_circles: the local center of circleA
   /// -e_faceA: the center of faceA
   /// -e_faceB: the center of faceB
   /// Similarly the local normal usage:
   /// -e_circles: not used
   /// -e_faceA: the normal on polygonA
   /// -e_faceB: the normal on polygonB
   /// We store contacts in this way so that position correction can
   /// account for movement, which is critical for continuous physics.
   /// All contact scenarios must be expressed in one of these types.
   /// This structure is stored across time steps, so we keep it small.
   Tb2ManifoldType = (e_manifold_circles, e_manifold_faceA, e_manifold_faceB);
   Pb2Manifold = ^Tb2Manifold;
   Tb2Manifold = record
      points: array[0..b2_maxManifoldPoints - 1] of Tb2ManifoldPoint; ///< the points of contact

      localNormal: TVector2;								///< not use for Type::e_points
      localPoint: TVector2;								///< usage depends on manifold type
      manifoldType: Tb2ManifoldType;
      pointCount: Int32;								///< the number of manifold points
   end;

   /// This is used to compute the current state of a contact manifold.
   Tb2WorldManifold = record
    	normal: TVector2;						///< world vector pointing from A to B
      points: array[0..b2_maxManifoldPoints - 1] of TVector2; ///< world contact point (point of intersection)

      {$IFDEF OP_OVERLOAD}
      /// Evaluate the manifold with supplied transforms. This assumes
      /// modest motion from the original state. This does not change the
      /// point count, impulses, etc. The radii must come from the shapes
      /// that generated the manifold.
      procedure Initialize(const manifold: Tb2Manifold; const xfA, xfB: Tb2Transform;
         radiusA, radiusB: Float);
      {$ENDIF}
   end;

   /// This is used for determining the state of contact points.
	 /// b2_nullState,		///< point does not exist
	 /// b2_addState,	   	///< point was added in the update
	 /// b2_persistState,	///< point persisted across the update
	 /// b2_removeState		///< point was removed in the update
   Tb2PointState = (b2_nullState, b2_addState, b2_persistState, b2_removeState);
   Tb2PointStateArray = array[0..b2_maxManifoldPoints - 1] of Tb2PointState;
   
   /// Ray-cast input data.
   Tb2RayCastInput = record
      p1, p2: TVector2;
      maxFraction: Float;
   end;

   /// Ray-cast output data.
   Tb2RayCastOutput = record
      normal: TVector2;
      fraction: Float;
   end;

   /// An axis aligned bounding box.
   Pb2AABB = ^Tb2AABB;
   Tb2AABB = record
      lowerBound, upperBound: TVector2; // The lower and upper vertices

      {$IFDEF OP_OVERLOAD}
    	function IsValid: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Verify that the bounds are sorted.
      function GetCenter: TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Get the center of the AABB.
      function GetExtents: TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Get the extents of the AABB (half-widths).
    	function GetPerimeter: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Get the perimeter length
      procedure Combine(const aabb: Tb2AABB); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Combine an AABB into this one.
      procedure Combine(const aabb1, aabb2: Tb2AABB); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Combine two AABBs into this one.
      function Contains(const aabb: Tb2AABB): Boolean; /// Does this aabb contain the provided AABB.
      function RayCast(var output: Tb2RayCastOutput; const input: Tb2RayCastInput): Boolean;
      {$ENDIF}
   end;

   Pb2PolyVertices = ^Tb2PolyVertices;
   Tb2PolyVertices = array[0..b2_maxPolygonVertices - 1] of TVector2;

   //////////////////////////////////////////////////////////////
   // World

   Tb2TimeStep = record
      dt: Float; // time step
      inv_dt: Float; // inverse time step (0 if dt == 0).
	    dtRatio: Float; // dt * inv_dt0
      velocityIterations, positionIterations: Int32;
      warmStarting: Boolean;
   end;

   /// Joints and fixtures are destroyed when their associated
   /// body is destroyed. Implement this listener so that you
   /// may nullify references to these joints and shapes.
   Tb2DestructionListener = class
   public
      /// Called when any fixtures is about to be destroyed due
      /// to the destruction of one of its attached bodies.
      procedure SayGoodbye(joint: Tb2Joint); overload; virtual; abstract;

      /// Called when any shape is about to be destroyed due
      /// to the destruction of its parent body.
      procedure SayGoodbye(fixture: Tb2Fixture); overload; virtual; abstract;
   end;

   Tb2DebugDrawBits = (e_shapeBit, e_jointBit, e_aabbBit, e_pairBit,
      e_centerOfMassBit{$IFDEF CONTROLLERS}, e_controllerBit{$ENDIF});
   Tb2DebugDrawBitsSet = set of Tb2DebugDrawBits;

   Tb2DebugDraw = class
   public
      m_drawFlags: Tb2DebugDrawBitsSet;
      m_shapeColor_Inactive, m_shapeColor_Static, m_shapeColor_Kinematic,
      m_shapeColor_Sleeping, m_shapeColor_Normal,
      m_pairColor, m_aabbColor, m_world_aabbColor, m_coreColor, m_jointLineColor: RGBA;

      constructor Create;

      procedure DrawPolygon(const vertices: Tb2PolyVertices; vertexCount: Int32; const color: RGBA); virtual; abstract;
      procedure DrawPolygon4(const vertices: TVectorArray4; vertexCount: Int32; const color: RGBA); virtual; abstract;
      procedure DrawSolidPolygon(const vertices: Tb2PolyVertices; vertexCount: Int32; const color: RGBA); virtual; abstract;
      procedure DrawCircle(const center: TVector2; radius: Float; const color: RGBA); virtual; abstract;
      procedure DrawSolidCircle(const center, axis: TVector2; radius: Float; const color: RGBA); virtual; abstract;
      procedure DrawSegment(const p1, p2: TVector2; const color: RGBA); virtual; abstract;
      procedure DrawTransform(const xf: Tb2Transform); virtual; abstract;
   end;

   // Simulate c++ template feature
   Tb2GenericCallBackWrapper = class
   public
      function QueryCallback(proxyId: Int32): Boolean; virtual; abstract;
      function RayCastCallback(const input: Tb2RayCastInput; proxyId: Int32): Float; virtual; abstract;
   end;

   /// The world class manages all physics entities, dynamic simulation,
   /// and asynchronous queries. The world also contains efficient memory
   /// management facilities.
   Tb2World = class
   private
      m_flags: UInt16;
      {$IFDEF COMPUTE_PHYSICSTIME}
      m_physicsTime: Double;
      {$ENDIF}

      m_contactManager: Tb2ContactManager;

      m_bodyList: Tb2Body;
      m_jointList: Tb2Joint;

      {$IFDEF CONTROLLERS}
      m_controllerList: Tb2Controller;
      m_controllerCount: Int32;
      {$ENDIF}

      m_bodyCount,
      m_jointCount: Int32;

      m_gravity: TVector2;
      m_allowSleep: Boolean;

      m_destructionListener: Tb2DestructionListener;
      m_debugDraw: Tb2DebugDraw;

      m_inv_dt0: Float;  // This is used to compute the time step ratio to support a variable time step.
      m_warmStarting: Boolean;
      m_continuousPhysics: Boolean; // For debugging the solver

	    m_subStepping: Boolean;
	    m_stepComplete: Boolean;

      procedure Solve(const step: Tb2TimeStep);

      { Sequentially solve TOIs for each body. We bring each body to the time
        of contact and perform some position correction. Time is not conserved.}
      procedure SolveTOI(const step: Tb2TimeStep);

      procedure DrawShape(fixture: Tb2Fixture; const xf: Tb2Transform; const color: RGBA);
      procedure DrawJoint(joint: Tb2Joint);

   public
      /// Construct a world object.
      /// @param gravity the world gravity vector.
      /// @param doSleep improve performance by not simulating inactive bodies.
      constructor Create(const gravity: TVector2; doSleep: Boolean);

      /// Destruct the world. All physics entities are destroyed and all heap memory is released.
      destructor Destroy; override;

      /// Register a contact filter to provide specific control over collision.
      /// Otherwise the default filter is used (b2_defaultFilter). The listener is
      /// owned by you and must remain in scope.
      procedure SetContactFilter(filter: Tb2ContactFilter);

      /// Register a contact event listener. The listener is owned by you and must remain in scope.
     	procedure SetContactListener(listener: Tb2ContactListener);

      /// Create a rigid body given a definition. No reference to the definition is retained.
      /// @warning This function is locked during callbacks.
      function CreateBody(def: Tb2BodyDef; AutoFreeBodyDef: Boolean = True): Tb2Body;

      /// Destroy a rigid body given a definition. No reference to the definition
      /// is retained. This function is locked during callbacks.
      /// @warning This automatically deletes all associated shapes and joints.
      /// @warning This function is locked during callbacks.
      procedure DestroyBody(body: Tb2Body; DoFree: Boolean = True);

      /// Create a joint to constrain bodies together. No reference to the definition
      /// is retained. This may cause the connected bodies to cease colliding.
      /// @warning This function is locked during callbacks.
      function CreateJoint(def: Tb2JointDef; AutoFreeJointDef: Boolean = True): Tb2Joint;

      /// Destroy a joint. This may cause the connected bodies to begin colliding.
      /// @warning This function is locked during callbacks.
      procedure DestroyJoint(j: Tb2Joint);

      {$IFDEF CONTROLLERS}
      procedure AddController(c: Tb2Controller);
      procedure RemoveController(c: Tb2Controller);
      {$ENDIF}

      /// Take a time step. This performs collision detection, integration,
      /// and constraint solution.
      /// @param timeStep the amount of time to simulate, this should not vary.
      /// @param velocityIterations for the velocity constraint solver.
      /// @param positionIterations for the position constraint solver.
      procedure Step(timeStep: Float; velocityIterations,
         positionIterations: Int32; Draw: Boolean = False);

      /// Call this after you are done with time steps to clear the forces. You normally
      /// call this after each call to Step, unless you are performing sub-steps. By default,
      /// forces will be automatically cleared, so you don't need to call this function.
      /// @see SetAutoClearForces
      procedure ClearForces;

      /// Call this to draw shapes and other debug draw data.
      procedure DrawDebugData;

      /// Query the world for all fixtures that potentially overlap the
      /// provided AABB.
      /// @param callback a user implemented callback class.
      /// @param aabb the query box.
      procedure QueryAABB(callback: Tb2QueryCallback; const aabb: Tb2AABB);

      /// Ray-cast the world for all fixtures in the path of the ray. Your callback
      /// controls whether you get the closest point, any point, or n-points.
      /// The ray-cast ignores shapes that contain the starting point.
      /// @param callback a user implemented callback class.
      /// @param point1 the ray starting point
      /// @param point2 the ray ending point
      procedure RayCast(callback: Tb2RayCastCallback; const point1, point2: TVector2);

      /// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
      /// the next contact in the world list. A NULL contact indicates the end of the list.
      /// @return the head of the world contact list.
      /// @warning contacts are
      function GetContactList: Pb2Contact; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function GetContactCount: Int32; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      function GetProxyCount: Int32; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}/// Get the number of broad-phase proxies.

      /// Change the global gravity vector.
      procedure SetGravity(const gravity: TVector2); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure WakeAllSleepingBodies;

      /// Is the world locked (in the middle of a time step).
      function IsLocked: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure SetAutoClearForces(flag: Boolean); /// Set flag to control automatic clearing of forces after each time step.

      /// Get the flag that controls automatic clearing of forces after each time step.
      function GetAutoClearForces: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      //////////////////////////////////////////////////////////////////////
      property DestructionListener: Tb2DestructionListener read m_destructionListener write m_destructionListener;
      property DebugDraw: Tb2DebugDraw read m_debugDraw write m_debugDraw;
      property GetContactManager: Tb2ContactManager read m_contactManager;

      property Gravity: TVector2 read m_gravity;
      property GetBodyList: Tb2Body read m_bodyList;
      property GetJointList: Tb2Joint read m_jointList;
      property GetBodyCount: Int32 read m_bodyCount;
      property GetJointCount: Int32 read m_jointCount;

      property WarmStarting: Boolean read m_warmStarting write m_warmStarting;
      property ContinuousPhysics: Boolean read m_continuousPhysics write m_continuousPhysics;

      property SubStepping: Boolean read m_subStepping write m_subStepping;

      {$IFDEF COMPUTE_PHYSICSTIME}
      property PhysicsTime: Double read m_physicsTime;
      {$ENDIF}
   end;

   ////////////////////////////////////////////////////
   // Contact

   /// A contact edge is used to connect bodies and contacts together
   /// in a contact graph where each body is a node and each contact
   /// is an edge. A contact edge belongs to a doubly linked list
   /// maintained in each attached body. Each contact has two contact
   /// nodes, one for each attached body.
   Pb2ContactEdge = ^Tb2ContactEdge;
   Tb2ContactEdge = record
      other: Tb2Body;	///< provides quick access to the other body attached.
      contact: Pb2Contact; ///< the contact
      prev, next: Pb2ContactEdge;
   end;

   Pb2ContactConstraintPoint = ^Tb2ContactConstraintPoint;
   Tb2ContactConstraintPoint = record
	    localPoint: TVector2;
      rA, rB: TVector2;

      normalImpulse, tangentImpulse, normalMass, tangentMass, velocityBias: Float;
   end;

   Pb2ContactConstraint = ^Tb2ContactConstraint;
   Tb2ContactConstraint = record
      points: array[0..b2_maxManifoldPoints - 1] of Tb2ContactConstraintPoint;
      localNormal, localPoint, normal: TVector2;
      normalMass, K: TMatrix22;

      manifold: Pb2Manifold;
      manifoldType: Tb2ManifoldType;

      bodyA, bodyB: Tb2Body;
      radiusA, radiusB: Float;
      restitution: Float;
      friction: Float;
      pointCount: Int32;
   end;

   /// The class manages contact between two shapes. A contact exists for each overlapping
   /// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
   /// that has no contact points.
   //Tb2ContactType = (b2_circleContact, b2_circlepolyContact, b2_polypolyContact);
   Tb2ContactEvaluateProc = procedure(contact: Pb2Contact; var manifold: Tb2Manifold;
      A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);

   Tb2Contact = record
      //m_type: Tb2ContactType;
      m_flags: UInt16;

      m_prev, m_next: Pb2Contact; // World pool and list pointers.
      m_nodeA, m_nodeB: Tb2ContactEdge; // Nodes for connecting bodies.
      m_fixtureA, m_fixtureB: Tb2Fixture;
      m_indexA, m_indexB: Int32;

      m_manifold: Tb2Manifold;
      m_toiCount: Int32;
      m_toi: Float;

      m_evaluateProc: Tb2ContactEvaluateProc; // For different contacts

      {$IFDEF OP_OVERLOAD}
      procedure Update(listener: Tb2ContactListener);
      /// Get the contact manifold. Do not modify the manifold unless you understand the
      /// internals of Box2D.
      function GetManifold: Pb2Manifold; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the world manifold.
      procedure GetWorldManifold(var worldManifold: Tb2WorldManifold); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

    	/// Flag this contact for filtering. Filtering will occur the next time step.
     	procedure FlagForFiltering; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Is this contact touching?
      function IsTouching: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Enable/disable this contact. This can be used inside the pre-solve
      /// contact listener. The contact is only disabled for the current
      /// time step (or sub-step in continuous collisions).
      procedure SetEnabled(flag: Boolean);
      function IsEnabled: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Has this contact been disabled?
      {$ENDIF}
   end;

   /// Contact impulses for reporting. Impulses are used instead of forces because
   /// sub-step forces may approach infinity for rigid body collisions. These
   /// match up one-to-one with the contact points in b2Manifold.
   Tb2ContactImpulse = record
      normalImpulses: array[0..b2_maxManifoldPoints - 1] of Float;
      tangentImpulses: array[0..b2_maxManifoldPoints - 1] of Float;
   end;

   /// Implement this class to provide collision filtering. In other words,
   /// you can implement this class if you want finer control over contact creation.
   Tb2ContactFilter = class
   public
      /// Return True if contact calculations should be performed between
      /// these two shapes. @warning for performance reasons this is only
      /// called when the AABBs begin to overlap.
      function ShouldCollide(fixtureA, fixtureB: Tb2Fixture): Boolean; virtual;
   end;

   /// Implement this class to get contact information. You can use these results for
   /// things like sounds and game logic. You can also get contact results by
   /// traversing the contact lists after the time step. However, you might miss
   /// some contacts because continuous physics leads to sub-stepping.
   /// Additionally you may receive multiple callbacks for the same contact in a
   /// single time step.
   /// You should strive to make your callbacks efficient because there may be
   /// many callbacks per time step.
   /// @warning You cannot create/destroy Box2D entities inside these callbacks.
   Tb2ContactListener = class(Tb2GenericCallBackWrapper)
   public
      /// Called when two fixtures begin to touch.
      procedure BeginContact(var contact: Tb2Contact); virtual;

      /// Called when two fixtures cease to touch.
      procedure EndContact(var contact: Tb2Contact); virtual;

      /// This is called after a contact is updated. This allows you to inspect a
      /// contact before it goes to the solver. If you are careful, you can modify the
      /// contact manifold (e.g. disable contact).
      /// A copy of the old manifold is provided so that you can detect changes.
      /// Note: this is called only for awake bodies.
      /// Note: this is called even when the number of contact points is zero.
      /// Note: this is not called for sensors.
      /// Note: if you set the number of contact points to zero, you will not
      /// get an EndContact callback. However, you may get a BeginContact callback
      /// the next step.
      procedure PreSolve(var contact: Tb2Contact; const oldManifold: Tb2Manifold); virtual;

      /// This lets you inspect a contact after the solver is finished. This is useful
      /// for inspecting impulses.
      /// Note: the contact manifold does not include time of impact impulses, which can be
      /// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
      /// in a separate data structure.
      /// Note: this is only called for contacts that are touching, solid, and awake.
      procedure PostSolve(var contact: Tb2Contact; const impulse: Tb2ContactImpulse); virtual;
   end;

   /// Callback class for AABB queries.
   /// See b2World::Query
   Tb2QueryCallback = class
   public
      /// Called for each fixture found in the query AABB.
      /// @return false to terminate the query.
      function ReportFixture(fixture: Tb2Fixture): Boolean; virtual; abstract;
   end;

   /// Callback class for ray casts. See b2World::RayCast
   Tb2RayCastCallback = class
   public
      /// Called for each fixture found in the query. You control how the ray cast
      /// proceeds by returning a float:
      /// return -1: ignore this fixture and continue
      /// return 0: terminate the ray cast
      /// return fraction: clip the ray to this point
      /// return 1: don't clip the ray and continue
      /// @param fixture the fixture hit by the ray
      /// @param point the point of initial intersection
      /// @param normal the normal vector at the point of intersection
      /// @return -1 to filter, 0 to terminate, fraction to clip the ray for
      /// closest hit, 1 to continue
      function ReportFixture(fixture:	Tb2Fixture; const point, normal: TVector2;
         fraction: Float): Float; virtual; abstract;
   end;

   Tb2ContactSolver = class
   public
      m_constraints: Pb2ContactConstraint;
      m_count: Int32;

      destructor Destroy; override;

      procedure Initialize(contacts: TList; count: Int32; impulseRatio: Float;
         warmStarting: Boolean);

      procedure InitializeVelocityConstraints;
      procedure WarmStart;
      procedure SolveVelocityConstraints;
      procedure StoreImpulses;

      function SolvePositionConstraints(baumgarte: Float): Boolean;
      function SolveTOIPositionConstraints(baumgarte: Float; toiBodyA, toiBodyB: Tb2Body): Boolean;
   end;

   // Delegate of b2World.
   Tb2ContactManager = class
   public
      m_broadPhase: Tb2BroadPhase;

      m_contactList: Pb2Contact;
      m_contactCount: Int32;

      m_contactFilter: Tb2ContactFilter;
      m_contactListener: Tb2ContactListener;

      constructor Create;
      destructor Destroy; overload; override;

      // Broad-phase callback.
      procedure AddPair(proxyUserDataA, proxyUserDataB: Pointer);
      procedure FindNewContacts; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure Destroy(pc: Pb2Contact); overload;
      procedure Collide;
   end;

   ////////////////////////////////////////////////////
   // Island

   /// This is an internal structure.
   Tb2Position = record
      x: TVector2;
      a: Float;
   end;

   /// This is an internal structure.
   Tb2Velocity = record
      v: TVector2;
      w: Float;
   end;

   Tb2Island = class
   private
      procedure Reset(bodyCapacity, contactCapacity, jointCapacity: Int32;
         listener: Tb2ContactListener); // Reset island solver so that instance won't be recreated over and over again
   public
      m_listener: Tb2ContactListener;

      m_bodies: TList;
      m_contacts: TList;
      m_joints: TList;

      m_positions: array of Tb2Position;
      m_velocities: array of Tb2Velocity;

      m_bodyCount: Int32;
      m_jointCount: Int32;
      m_contactCount: Int32;

      m_bodyCapacity, m_contactCapacity, m_jointCapacity: Int32;

      constructor Create;
      destructor Destroy; override;

      procedure Clear;
      procedure Solve(const step: Tb2TimeStep; const gravity: TVector2; allowSleep: Boolean);
      procedure SolveTOI(const subStep: Tb2TimeStep; bodyA, bodyB: Tb2Body);

      procedure Add(body: Tb2Body); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure Add(contact: Pb2Contact); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      procedure Add(joint: Tb2Joint); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      procedure Report(constraints: Pb2ContactConstraint);
   end;

   Pb2TOIConstraint = ^Tb2TOIConstraint;
   Tb2TOIConstraint = record
      localPoints: array[0..b2_maxManifoldPoints - 1] of TVector2;
      localNormal, localPoint: TVector2;
      manifoldType: Tb2ManifoldType;
      radius: Float;
      pointCount: Int32;
      bodyA, bodyB: Tb2Body;
   end;

   ////////////////////////////////////////////////////
   Pb2Pair = ^Tb2Pair;
   Tb2Pair = record
      proxyIdA,
      proxyIdB,
      next: Int32;
   end;

   Tb2DynamicTreeNode = record
   {$IFDEF OP_OVERLOAD}
   public
      function IsLeaf: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
   public
   {$ENDIF}
    	aabb: Tb2AABB; /// This is the fattened AABB.
      userData: Pointer;
      child1, child2: Int32;
      leafCount: Int32;
      case Byte of
         0: (parent: Int32);
         1: (next: Int32);
   end;

   /// A dynamic tree arranges data in a binary tree to accelerate
   /// queries such as volume queries and ray casts. Leafs are proxies
   /// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
   /// so that the proxy AABB is bigger than the client object. This allows the client
   /// object to move by small amounts without triggering a tree update.
   ///
   /// Nodes are pooled and relocatable, so we use node indices rather than pointers.
   Tb2DynamicTree = class
   private
      m_root: Int32;
      m_nodes: array of Tb2DynamicTreeNode;
      m_nodeCount, m_nodeCapacity: Int32;
      m_freeList: Int32;
      m_path: UInt32; /// This is used incrementally traverse the tree for re-balancing.
      m_insertionCount: Int32;

      function AllocateNode: Int32;
      procedure FreeNode(nodeId: Int32);

      procedure InsertLeaf(leaf: Int32);
      procedure RemoveLeaf(leaf: Int32);

      function ComputeHeight(nodeId: Int32): Int32; overload;
      function CountLeaves(nodeId: Int32): Int32;
   public
      constructor Create;

      procedure SetCapacity(value: Int32);

      /// Create a proxy. Provide a tight fitting AABB and a userData pointer.
      function CreateProxy(const _aabb: Tb2AABB; _userData: Pointer): Int32;

      /// Destroy a proxy. This asserts if the id is invalid.
      procedure DestroyProxy(proxyId: Int32); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
      /// then the proxy is removed from the tree and re-inserted. Otherwise
      /// the function returns immediately.
      /// @return true if the proxy was re-inserted.
      function MoveProxy(proxyId: Int32; const aabb: Tb2AABB;
         const displacement: TVector2): Boolean;

      /// Perform some iterations to re-balance the tree.
      procedure Rebalance(iterations: Int32);

      /// Get proxy user data.
      /// @return the proxy user data or 0 if the id is invalid.
      function GetUserData(proxyId: Int32): Pointer; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the fat AABB for a proxy.
      function GetFatAABB(proxyId: Int32): Pb2AABB; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

    	/// Compute the height of the binary tree in O(N) time. Should not be called often.
      function ComputeHeight: Int32; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Query an AABB for overlapping proxies. The callback class
      /// is called for each proxy that overlaps the supplied AABB.
      procedure Query(callback: Tb2GenericCallBackWrapper; const _aabb: Tb2AABB);

      /// Ray-cast against the proxies in the tree. This relies on the callback
      /// to perform a exact ray-cast in the case were the proxy contains a shape.
      /// The callback also performs the any collision filtering. This has performance
      /// roughly equal to k * log(n), where k is the number of collisions and n is the
      /// number of proxies in the tree.
      /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
      /// @param callback a callback class that is called for each proxy that is hit by the ray.
      procedure RayCast(callback: Tb2GenericCallBackWrapper; const input: Tb2RayCastInput);

      procedure Validate; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
   end;

   /// The broad-phase is used for computing pairs and performing volume queries and ray casts.
   /// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
   /// It is up to the client to consume the new pairs and to track subsequent overlap.
   Tb2BroadPhase = class(Tb2GenericCallBackWrapper)
   private
      m_tree: Tb2DynamicTree;

      m_proxyCount: Int32;
      m_moveBuffer: array of Int32;
      m_moveCapacity,
      m_moveCount: Int32;

      m_pairBuffer: array of Tb2Pair;
      m_pairCapacity,
      m_pairCount: Int32;

      m_queryProxyId: Int32;

      procedure QuickSortPairBuffer(L, R: Int32);
      procedure BufferMove(proxyId: Int32);
      procedure UnBufferMove(proxyId: Int32);

      function QueryCallback(proxyId: Int32): Boolean; override;
   public
      constructor Create;
      destructor Destroy; override;

      /// Create a proxy with an initial AABB. Pairs are not reported until
      /// UpdatePairs is called.
      function CreateProxy(const aabb: Tb2AABB; userData: Pointer): Int32; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Destroy a proxy. It is up to the client to remove any pairs.
      procedure DestroyProxy(proxyId: Int32); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Call MoveProxy as many times as you like, then when you are done
      /// call UpdatePairs to finalized the proxy pairs (for your time step).
      procedure MoveProxy(proxyId: Int32; const aabb: Tb2AABB; const displacement: TVector2); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the fat AABB for a proxy.
      function GetFatAABB(proxyId: Int32): Pb2AABB; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get user data from a proxy. Returns NULL if the id is invalid.
      function GetUserData(proxyId: Int32): Pointer; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Test overlap of fat AABBs.
      function TestOverlap(proxyIdA, proxyIdB: Int32): Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Update the pairs. This results in pair callbacks. This can only add pairs.
      procedure UpdatePairs(callback: Tb2ContactManager);

      /// Query an AABB for overlapping proxies. The callback class
      /// is called for each proxy that overlaps the supplied AABB.
      procedure Query(callback: Tb2GenericCallBackWrapper; const aabb: Tb2AABB); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Ray-cast against the proxies in the tree. This relies on the callback
      /// to perform a exact ray-cast in the case were the proxy contains a shape.
      /// The callback also performs the any collision filtering. This has performance
      /// roughly equal to k * log(n), where k is the number of collisions and n is the
      /// number of proxies in the tree.
      /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
      /// @param callback a callback class that is called for each proxy that is hit by the ray.
      procedure RayCast(callback: Tb2GenericCallBackWrapper; const input: Tb2RayCastInput); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      function ComputeHeight: Int32; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Compute the height of the embedded tree.

      property GetProxyCount: Int32 read m_proxyCount; /// Get the number of proxies.
   end;

   //////////////////////////////////////////////////////////////
   /// Fixture & Shapes

   /// This holds the mass data computed for a shape.
   Tb2MassData = record
	    mass: Float; /// The mass of the shape, usually in kilograms.
      I: Float; /// The rotational inertia of the shape about the local origin.
      center: TVector2; /// The position of the shape's centroid relative to the shape's origin.
   end;

   /// The various collision shape types supported by Box2D.
   Tb2ShapeType = (e_unknownShape = -1, e_circleShape, e_edgeShape,
      e_polygonShape, e_loopShape);

   /// A shape is used for collision detection. You can create a shape however you like.
   /// Shapes used for simulation in b2World are created automatically when a b2Fixture
   /// is created. Shapes may encapsulate a one or more child shapes.
   Tb2Shape = class
   private
      m_type: Tb2ShapeType;
      m_fixture: Tb2Fixture;
      m_destroyed: Boolean;
      m_baseMass: Tb2MassData; // Density = 1
   public      
      m_radius: Float;

      constructor Create;
      destructor Destroy; override;

      /// Clone the concrete shape using the provided allocator.
      function Clone: Tb2Shape; virtual; abstract;

     	/// Get the number of child primitives.
	    function GetChildCount: Int32; virtual; abstract;

      /// Test a point for containment in this shape. This only works for convex shapes.
      /// @param xf the shape world transform.
      /// @param p a point in world coordinates.
      function TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean; virtual; abstract;

      /// Cast a ray against a child shape.
      /// @param output the ray-cast results.
      /// @param input the ray-cast input parameters.
      /// @param transform the transform to be applied to the shape.
      /// @param childIndex the child shape index
      function RayCast(var output: Tb2RayCastOutput; const input: Tb2RayCastInput;
         const transform: Tb2Transform; childIndex: Int32): Boolean; virtual; abstract;

      /// Given a transform, compute the associated axis aligned bounding box for a child shape.
      /// @param aabb returns the axis aligned box.
      /// @param xf the world transform of the shape.
      /// @param childIndex the child shape
      procedure ComputeAABB(var aabb: Tb2AABB; const xf: Tb2Transform; childIndex: Int32); virtual; abstract;

      /// Compute the mass properties of this shape using its dimensions and density.
      /// The inertia tensor is computed about the local origin.
      /// @param massData returns the mass data for this shape.
      /// @param density the density in kilograms per meter squared.
      procedure ComputeMass(var massData: Tb2MassData; density: Float); virtual; abstract;

      { * Compute the volume and centroid of this shape intersected with a half plane
        * @param normal the surface normal
        * @param offset the surface offset along normal
        * @param xf the shape transform
        * @param c returns the centroid
        * @return the total volume less than offset along normal }
      function ComputeSubmergedArea(const normal: TVector2; offset: Float;
         const xf: Tb2Transform; var c: TVector2): Float; virtual; abstract;
      property GetType: Tb2ShapeType read m_type;
   end;

   /// This holds contact filtering data.
   Pb2Filter = ^Tb2Filter;
   Tb2Filter = record
      /// The collision category bits. Normally you would just set one bit.
      categoryBits: UInt16;

      /// The collision mask bits. This states the categories that this
      /// shape would accept for collision.
      maskBits: UInt16;

      /// Collision groups allow a certain group of objects to never collide (negative)
      /// or always collide (positive). Zero means no collision group. Non-zero group
      /// filtering always wins against the mask bits.
      groupIndex: Int16;
   end;

   /// This proxy is used internally to connect fixtures to the broad-phase.
   Pb2FixtureProxy = ^Tb2FixtureProxy;
   Tb2FixtureProxy = record
      aabb: Tb2AABB;
      fixture: Tb2Fixture;
      childIndex: Int32;
      proxyId: Int32;
   end;

   /// A fixture definition is used to create a fixture. This class defines an
   /// abstract fixture definition. You can reuse fixture definitions safely.
   Tb2FixtureDef = class
   public
      /// The shape, this must be set. The shape will be cloned, so you
      /// can create the shape on the stack.
      shape: Tb2Shape;
      userData: Pointer; /// Use this to store application specific fixture data.
      friction: Float; /// The friction coefficient, usually in the range [0,1].
      restitution: Float; /// The restitution (elasticity) usually in the range [0,1].
      density: Float; /// The density, usually in kg/m^2.
      isSensor: Boolean; /// A sensor shape collects contact information but never generates a collision response.
      filter: Tb2Filter; /// Contact filtering data.

      constructor Create;
   end;

   Tb2Fixture = class
   private
      m_shape: Tb2Shape;
      destructor Destroy2;
   protected
      m_density: Float;

      m_next: Tb2Fixture;
      m_body: Tb2Body;

      m_friction,
      m_restitution: Float;

      m_proxies: array of Tb2FixtureProxy;
      m_proxyCount: Int32;

      m_filter: Tb2Filter;

      m_isSensor: Boolean;
      m_userData: Pointer;

      // These support body activation/deactivation.
      procedure CreateProxies(broadPhase: Tb2BroadPhase; const xf: Tb2Transform);
      procedure DestroyProxies(broadPhase: Tb2BroadPhase);
      procedure Synchronize(broadPhase: Tb2BroadPhase; const xf1, xf2: Tb2Transform);

   public
      constructor Create(body: Tb2Body; def: Tb2FixtureDef; AutoFreeShape: Boolean = True);
      destructor Destroy; override;

      /// Get the type of the child shape. You can use this to down cast to the concrete shape.
	    function GetType: Tb2ShapeType; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Set the contact filtering data. This will not update contacts until the next time
      /// step when either parent body is active and awake.
      procedure SetFilterData(const filter: Tb2Filter);
      function GetFilterData: Pb2Filter; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Test a point for containment in this fixture.
      /// @param xf the shape world transform.
      /// @param p a point in world coordinates.
      function TestPoint(const p: TVector2): Boolean;

      /// Cast a ray against this shape.
      /// @param output the ray-cast results.
      /// @param input the ray-cast input parameters.
      function RayCast(var output: Tb2RayCastOutput; const input: Tb2RayCastInput; childIndex: Int32): Boolean;

      /// Get the mass data for this fixture. The mass data is based on the density and
      /// the shape. The rotational inertia is about the shape's origin. This operation
      /// may be expensive.
      procedure GetMassData(var massData: Tb2MassData);

      /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
      /// If you need a more accurate AABB, compute it using the shape and
      /// the body transform.
      function GetAABB(childIndex: Int32): Pb2AABB; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      property GetShape: Tb2Shape read m_shape;
      property GetBody: Tb2Body read m_body;
      property GetNext: Tb2Fixture read m_next;
      property IsSensor: Boolean read m_isSensor write m_isSensor;
      property UserData: Pointer read m_userData write m_userData;
      property Density: Float read m_density write m_density;
      property Friction: Float read m_friction write m_friction;
      property Restitution: Float read m_restitution write m_restitution;
   end;

   //////////////////////////////////////////////////////////////
   // Joints    
   Tb2JointType = (e_unknownJoint, e_revoluteJoint, e_prismaticJoint, 
      e_distanceJoint, e_pulleyJoint, e_mouseJoint, e_gearJoint, e_lineJoint,
      e_weldJoint, e_frictionJoint, e_fixedJoint);
   Tb2LimitState = (e_inactiveLimit, e_atLowerLimit, e_atUpperLimit, e_equalLimits);

   Tb2Jacobian = record
      linearA, linearB: TVector2;
      angularA, angularB: Float;

      {$IFDEF OP_OVERLOAD}
      procedure SetZero;
      procedure SetValue(const x1, x2: TVector2; a1, a2: Float);
      function Compute(const x1, x2: TVector2; a1, a2: Float): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      {$ENDIF}      
   end;

   /// A joint edge is used to connect bodies and joints together
   /// in a joint graph where each body is a node and each joint
   /// is an edge. A joint edge belongs to a doubly linked list
   /// maintained in each attached body. Each joint has two joint
   /// nodes, one for each attached body.
   Pb2JointEdge = ^Tb2JointEdge;
   Tb2JointEdge = record
      other: Tb2Body; ///< provides quick access to the other body attached.
      joint: Tb2Joint; ///< the joint
      prev, next: Pb2JointEdge;
   end;

   /// Joint definitions are used to construct joints.
   Tb2JointDef = class
   public            
      JointType: Tb2JointType; /// The joint type is set automatically for concrete joint types.
      userData: Pointer; /// Use this to attach application specific data to your joints.

      bodyA, bodyB: Tb2Body ; /// The attached bodies.
      collideConnected: Boolean; /// Set this flag to True if the attached bodies should collide.

      constructor Create;
   end;  

   /// The base joint class. Joints are used to constraint two bodies together in
   /// various fashions. Some joints also feature limits and motors.
   Tb2Joint = class
   private
      // Cache here per time step to reduce cache misses.
      m_localCenterA, m_localCenterB: TVector2;
      m_invMassA, m_invIA,
      m_invMassB, m_invIB: Float;
   protected
      m_type: Tb2JointType;
      m_prev, m_next: Tb2Joint;
      m_edgeA, m_edgeB: Tb2JointEdge;
      m_bodyA, m_bodyB: Tb2Body;

      m_islandFlag, m_collideConnected: Boolean;
      m_userData: Pointer;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); virtual; abstract;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); virtual; abstract;

      // This returns True if the position errors are within tolerance.
      function SolvePositionConstraints(baumgarte: Float): Boolean; virtual; abstract;

   public
      constructor Create(def: Tb2JointDef);

      /// Get the anchor point on bodyA in world coordinates.
      function GetAnchorA: TVector2; virtual; abstract;

      /// Get the anchor point on bodyB in world coordinates.
      function GetAnchorB: TVector2; virtual; abstract;

      /// Get the reaction force on bodyB at the joint anchor in Newtons.
      function GetReactionForce(inv_dt: Float): TVector2; virtual; abstract;

      /// Get the reaction torque on bodyB in N*m.
      function GetReactionTorque(inv_dt: Float): Float; virtual; abstract;

	    /// Short-cut function to determine if either body is inactive.
    	function IsActive: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      property GetType: Tb2JointType read m_type;
      property GetBodyA: Tb2Body read m_bodyA;
      property GetBodyB: Tb2Body read m_bodyB;
      property GetNext: Tb2Joint read m_next;
      property UserData: Pointer read m_userData write m_userData;
   end;

   {$IFDEF CONTROLLERS}
   //////////////////////////////////////////////////////////////
   // Controllers

   /// A controller edge is used to connect bodies and controllers together
   /// in a bipartite graph.
   Pb2ControllerEdge = ^Tb2ControllerEdge;
   Tb2ControllerEdge = record
      controller: Tb2Controller;		///< provides quick access to other end of this edge.
      body: Tb2Body;					///< the body
      prevBody: Pb2ControllerEdge;		///< the previous controller edge in the controllers's joint list
      nextBody: Pb2ControllerEdge;		///< the next controller edge in the controllers's joint list
      prevController: Pb2ControllerEdge;		///< the previous controller edge in the body's joint list
      nextController: Pb2ControllerEdge;		///< the next controller edge in the body's joint list
   end;

   /// Base class for controllers. Controllers are a convience for encapsulating common
   /// per-step functionality.
   Tb2Controller = class
   private
      m_prev,
      m_next: Tb2Controller;
   protected
      m_world: Tb2World;

      m_bodyList: Pb2ControllerEdge;
      m_bodyCount: Int32;
   public
      destructor Destroy; override;

      /// Controllers override this to implement per-step functionality.
      procedure Step(const step: Tb2TimeStep); virtual; abstract;

      /// Controllers override this to provide debug drawing.
      procedure Draw(debugDraw: Tb2DebugDraw); virtual;

      /// Adds a body to the controller list.
      procedure AddBody(body: Tb2Body); virtual;

      /// Removes a body from the controller list.
      procedure RemoveBody(body: Tb2Body); virtual;

      /// Removes all bodies from the controller list.
      procedure Clear;

      property GetNext: Tb2Controller read m_next; /// Get the next controller in the world's body list.
      property GetWorld: Tb2World read m_world;/// Get the parent world of this body.
      property GetBodyList: Pb2ControllerEdge read m_bodyList; /// Get the attached body list
   end;
   {$ENDIF}

   //////////////////////////////////////////////////////////////
   // Body

   /// The body type.
   /// static: zero mass, zero velocity, may be manually moved
   /// kinematic: zero mass, non-zero velocity set by user, moved by solver
   /// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
   Tb2BodyType = (b2_staticBody, b2_kinematicBody, b2_dynamicBody);

   /// A body definition holds all the data needed to construct a rigid body.
   /// You can safely re-use body definitions.
   Tb2BodyDef = class
   public
      bodyType: Tb2BodyType; /// Note: if a dynamic body would have zero mass, the mass is set to one.
      userData: Pointer; /// Use this to store application specific body data.

      ignoreColliding: Boolean;
      /// The world position of the body. Avoid creating bodies at the origin
      /// since this can lead to many overlapping shapes.
      position: TVector2;

      angle: Float; // The world angle of the body in radians.

    	/// The linear velocity of the body's origin in world co-ordinates.
	    linearVelocity: TVector2;

    	angularVelocity: Float; /// The angular velocity of the body.

      /// Linear damping is use to reduce the linear velocity. The damping parameter
      /// can be larger than 1.0f but the damping effect becomes sensitive to the
      /// time step when the damping parameter is large.
      linearDamping: Float;

      /// Angular damping is use to reduce the angular velocity. The damping parameter
      /// can be larger than 1.0f but the damping effect becomes sensitive to the
      /// time step when the damping parameter is large.
      angularDamping: Float;

      /// Set this flag to false if this body should never fall asleep. Note that
      /// this increases CPU usage.
      allowSleep: Boolean;

      awake: Boolean; /// Is this body initially awake or sleeping?
      fixedRotation: Boolean; /// Should this body be prevented from rotating? Useful for characters.

      /// Is this a fast moving body that should be prevented from tunneling through
      /// other moving bodies? Note that all bodies are prevented from tunneling through
      /// kinematic and static bodies. This setting is only considered on dynamic bodies.
      /// @warning You should use this flag sparingly since it increases processing time.
      bullet: Boolean;

      active: Boolean; /// Does this body start out active?

	    inertiaScale: Float; /// Experimental: scales the inertia tensor.

      constructor Create;
   end;

   Tb2Body = class
   private
      m_type: Tb2BodyType;

      m_flags: UInt16;
      m_islandIndex: Int32;

      m_world: Tb2World;
      m_prev, m_next: Tb2Body;

      m_fixtureList: Tb2Fixture;
      m_fixtureCount: Int32;

      m_jointList: Pb2JointEdge;
      m_contactList: Pb2ContactEdge;

      {$IFDEF CONTROLLERS}
	    m_controllerList: Pb2ControllerEdge;
	    m_controllerCount: Int32;
      {$ENDIF}

      m_mass, m_invMass: Float;
      m_I, m_invI: Float; // Rotational inertia about the center of mass.
      m_storedInertia: Float;

      m_linearDamping: Float;
      m_angularDamping: Float;

      destructor Destroy2; // Only free heap
      procedure ComputeStoredInertia;

      procedure SynchronizeFixtures;
      procedure SynchronizeTransform;

	    // This is used to prevent connected bodies from colliding.
	    // It may lie, depending on the collideConnected flag.
	    function ShouldCollide(other: Tb2Body): Boolean;
      procedure Advance(alpha: Float);
   protected
      m_xf: Tb2Transform; // the body origin transform
      m_sweep: Tb2Sweep; // the swept motion for CCD

      m_linearVelocity: TVector2;
      m_angularVelocity: Float;

      m_force: TVector2;
      m_torque: Float;

      m_sleepTime: Float;
      m_userData: Pointer;
   public
      constructor Create(bd: Tb2BodyDef; world: Tb2World);
      destructor Destroy; override;

      /// Creates a fixture and attach it to this body. Use this function if you need
      /// to set some fixture parameters, like friction. Otherwise you can create the
      /// fixture directly from a shape.
      /// If the density is non-zero, this function automatically updates the mass of the body.
      /// Contacts are not created until the next time step.
      /// @param def the fixture definition.
      /// @warning This function is locked during callbacks.
      function CreateFixture(def: Tb2FixtureDef; AutoFreeFixtureDef: Boolean = True;
         AutoFreeShape: Boolean = True; AutoResetMassData: Boolean = True): Tb2Fixture; overload;

      /// Creates a fixture from a shape and attach it to this body.
      /// This is a convenience function. Use b2FixtureDef if you need to set parameters
      /// like friction, restitution, user data, or filtering.
      /// If the density is non-zero, this function automatically updates the mass of the body.
      /// @param shape the shape to be cloned.
      /// @param density the shape density (set to zero for static bodies).
      /// @warning This function is locked during callbacks.
      function CreateFixture(shape: Tb2Shape; density: Float;
         AutoFreeShape: Boolean = True; AutoResetMassData: Boolean = True): Tb2Fixture; overload;

      /// Destroy a fixture. This removes the fixture from the broad-phase and
      /// destroys all contacts associated with this fixture. This will
      /// automatically adjust the mass of the body if the body is dynamic and the
      /// fixture has positive density.
      /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
      /// @param fixture the fixture to be removed.
      /// @warning This function is locked during callbacks.
      procedure DestroyFixture(fixture: Tb2Fixture; DoFree: Boolean = True);

      /// Destroy all fixtures. If ResetMass is False, Self becomes a virtual body
      /// which doesn't react to collisions but keep all physical features.
      procedure DestroyFixtures(ResetMass: Boolean);

      /// Set the position of the body's origin and rotation.
      /// This breaks any contacts and wakes the other bodies.
      /// Manipulating a body's transform may cause non-physical behavior.
      /// @param position the world position of the body's local origin.
      /// @param angle the world rotation in radians.
      procedure SetTransform(const position: TVector2; angle: Float);
    	/// Get the body transform for the body's origin.
    	/// @return the world transform of the body's origin.
    	function GetTransform: Pb2Transform; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Set the linear velocity of the center of mass.
      /// @param v the new linear velocity of the center of mass.
      procedure SetLinearVelocity(const v: TVector2); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Set the angular velocity.
      /// @param omega the new angular velocity in radians/second.
      procedure SetAngularVelocity(omega: Float); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Apply a force at a world point. If the force is not
      /// applied at the center of mass, it will generate a torque and
      /// affect the angular velocity. This wakes up the body.
      /// @param force the world force vector, usually in Newtons (N).
      /// @param point the world position of the point of application.
      procedure ApplyForce(const force, point: TVector2);

      /// Apply a torque. This affects the angular velocity
      /// without affecting the linear velocity of the center of mass.
      /// This wakes up the body.
      /// @param torque about the z-axis (out of the screen), usually in N-m.
      procedure ApplyTorque(torque: Float);

      /// Apply an impulse at a point. This immediately modifies the velocity.
      /// It also modifies the angular velocity if the point of application
      /// is not at the center of mass. This wakes up the body.
      /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
      /// @param point the world position of the point of application.
      procedure ApplyLinearImpulse(const impulse, point: TVector2);
      /// Apply an angular impulse.
      /// @param impulse the angular impulse in units of kg*m*m/s
      procedure ApplyAngularImpulse(impulse: Float);

    	/// Get the mass data of the body.
	    /// @return a struct containing the mass, inertia and center of the body.
      procedure GetMassData(var data: Tb2MassData);

      /// Set the mass properties to override the mass properties of the fixtures.
      /// Note that this changes the center of mass position.
      /// Note that creating or destroying fixtures can also alter the mass.
      /// This function has no effect if the body isn't dynamic.
      /// @param massData the mass properties.
      procedure SetMassData(const data: Tb2MassData);

      /// This resets the mass properties to the sum of the mass properties of the fixtures.
      /// This normally does not need to be called unless you called SetMassData to override
      /// the mass and you later want to reset the mass.
      procedure ResetMassData;

      /// Get the world coordinates of a point given the local coordinates.
      /// @param localPoint a point on the body measured relative the the body's origin.
      /// @return the same point expressed in world coordinates.
      function GetWorldPoint(const localPoint: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the world coordinates of a vector given the local coordinates.
      /// @param localVector a vector fixed in the body.
      /// @return the same vector expressed in world coordinates.
      function GetWorldVector(const localVector: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Gets a local point relative to the body's origin given a world point.
      /// @param a point in world coordinates.
      /// @return the corresponding local point relative to the body's origin.
      function GetLocalPoint(const worldPoint: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Gets a local vector given a world vector.
      /// @param a vector in world coordinates.
      /// @return the corresponding local vector.
      function GetLocalVector(const worldVector: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the world linear velocity of a world point attached to this body.
      /// @param a point in world coordinates.
      /// @return the world velocity of a point.
      function GetLinearVelocityFromWorldPoint(const worldPoint: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the world velocity of a local point.
      /// @param a point in local coordinates.
      /// @return the world velocity of a point.
      function GetLinearVelocityFromLocalPoint(const localPoint: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Set the type of this body. This may alter the mass and velocity.
    	procedure SetType(atype: Tb2BodyType);

      /// Is this body treated like a bullet for continuous collision detection?
      function IsBullet: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Should this body be treated like a bullet for continuous collision detection?
      procedure SetBullet(flag: Boolean);

      /// You can disable sleeping on this body. If you disable sleeping, the
      /// body will be woken.
      procedure SetSleepingAllowed(flag: Boolean);

      /// Is this body allowed to sleep
      function IsSleepingAllowed: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Set the sleep state of the body. A sleeping body has very
      /// low CPU cost.
      /// @param flag set to True to put body to sleep, false to wake it.
      procedure SetAwake(flag: Boolean);

      /// Get the sleeping state of this body.
      /// @return True if the body is sleeping.
      function IsAwake: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Set the active state of the body. An inactive body is not
      /// simulated and cannot be collided with or woken up.
      /// If you pass a flag of True, all fixtures will be added to the
      /// broad-phase.
      /// If you pass a flag of false, all fixtures will be removed from
      /// the broad-phase and all contacts will be destroyed.
      /// Fixtures and joints are otherwise unaffected. You may continue
      /// to create/destroy fixtures and joints on inactive bodies.
      /// Fixtures on an inactive body are implicitly inactive and will
      /// not participate in collisions, ray-casts, or queries.
      /// Joints connected to an inactive body are implicitly inactive.
      /// An inactive body is still owned by a b2World object and remains
      /// in the body list.
      procedure SetActive(flag: Boolean);

      /// Get the active state of the body.
      function IsActive: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      procedure SetIgnoreColliding(flag: Boolean); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function IsCollidingIgnored: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Set this body to have fixed rotation. This causes the mass
      /// to be reset.
      procedure SetFixedRotation(flag: Boolean);

      /// Does this body have fixed rotation?
      function IsFixedRotation: Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      ////////////////////////////////////
      property GetType: Tb2BodyType read m_type;
      property GetAngle: Float read m_sweep.a;
      property GetPosition: TVector2 read m_xf.position;
      property GetWorldCenter: TVector2 read m_sweep.c;
      property GetLocalCenter: TVector2 read m_sweep.localCenter;
      property GetLinearVelocity: TVector2 read m_linearVelocity;
      property GetAngularVelocity: Float read m_angularVelocity;
      property LinearDamping: Float read m_linearDamping write m_linearDamping;
      property AngularDamping: Float read m_angularDamping write m_angularDamping;

      property GetMass: Float read m_mass;
      /// Get the rotational inertia of the body about the local origin.
      /// @return the rotational inertia, usually in kg-m^2.
      property GetInertia: Float read m_storedInertia;

      property GetFixtureList: Tb2Fixture read m_fixtureList;
      property GetJointList: Pb2JointEdge read m_jointList;
      property GetContactList: Pb2ContactEdge read m_contactList;
      property GetNext: Tb2Body read m_next;
      property UserData: Pointer read m_userData write m_userData;
      property GetWorld: Tb2World read m_world;
      {$IFDEF CONTROLLERS}
      property GetControllerCount: Int32 read m_controllerCount;
      property GetControllerList: Pb2ControllerEdge read m_controllerList;
      {$ENDIF}
   end;

   ///////////////////////////////////////////////
   // Specific implementations

   Tb2CircleShape = class(Tb2Shape)
   public
      m_p: TVector2;

      constructor Create;

      function Clone: Tb2Shape; override;
      function GetChildCount: Int32; override;
      function TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean; override;
      function RayCast(var output: Tb2RayCastOutput; const input: Tb2RayCastInput;
         const transform: Tb2Transform; childIndex: Int32): Boolean; override;
      procedure ComputeAABB(var aabb: Tb2AABB; const xf: Tb2Transform; childIndex: Int32); override;
      procedure ComputeMass(var massData: Tb2MassData; density: Float); override;
      function ComputeSubmergedArea(const normal: TVector2; offset: Float;
         const xf: Tb2Transform; var c: TVector2): Float; override;

      /// Get the supporting vertex index in the given direction.
      function GetSupport(const d: TVector2): Int32; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the supporting vertex in the given direction.
      function GetSupportVertex(const d: TVector2): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the vertex count.
      function GetVertexCount: Int32; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get a vertex by index. Used by b2Distance.
      function GetVertex(index: Int32): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
   end;

   /// A convex polygon.
   Tb2PolygonShape = class(Tb2Shape)
   public
      m_centroid: TVector2; // Local position of the polygon centroid.
      m_vertices: Tb2PolyVertices;
      m_normals: Tb2PolyVertices;
      m_vertexCount: Int32;
      m_edgeShapeMassed: Boolean; // If true density is linear density and mass will not be zero.

      constructor Create;

      function Clone: Tb2Shape; override;
      function GetChildCount: Int32; override;
      function TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean; override;
      function RayCast(var output: Tb2RayCastOutput; const input: Tb2RayCastInput;
         const transform: Tb2Transform; childIndex: Int32): Boolean; override;
      procedure ComputeAABB(var aabb: Tb2AABB; const xf: Tb2Transform; childIndex: Int32); override;
      procedure ComputeMass(var massData: Tb2MassData; density: Float); override;
      function ComputeSubmergedArea(const normal: TVector2; offset: Float;
         const xf: Tb2Transform; var c: TVector2): Float; override;

      /// Copy vertices. This assumes the vertices define a convex polygon.
      /// It is assumed that the exterior is the the right of each edge.
      procedure SetVertices(vertices: PVector2; count: Int32);

      /// Build vertices to represent an axis-aligned box.
      /// @param hx the half-width.
      /// @param hy the half-height.
      procedure SetAsBox(hx, hy: Float); overload;

      /// Build vertices to represent an oriented box.
      /// @param hx the half-width.
      /// @param hy the half-height.
      /// @param center the center of the box in local coordinates.
      /// @param angle the rotation of the box in local coordinates.
      procedure SetAsBox(hx, hy: Float; const center: TVector2; angle: Float); overload;

      /// Set this as a single edge.
      procedure SetAsEdge(const v1, v2: TVector2);

      /// Get the supporting vertex index in the given direction.
      //function GetSupport(const d: TVector2): Int32;
	    /// Get the supporting vertex in the given direction.
    	//function GetSupportVertex(const d: TVector2): PVector2;

      property GetVertexCount: Int32 read m_vertexCount;
   end;

   /// A line segment (edge) shape. These can be connected in chains or loops
   /// to other edge shapes. The connectivity information is used to ensure
   /// correct contact normals.
   Tb2EdgeShape = class(Tb2Shape)
   public
      m_vertex1, m_vertex2: TVector2; /// These are the edge vertices
	    /// Optional adjacent vertices. These are used for smooth collision.
    	m_vertex0, m_vertex3: TVector2;
	    m_hasVertex0, m_hasVertex3: Boolean;

      constructor Create;

      procedure SetVertices(const v1, v2: TVector2); // implements C++ "Set" function
      function Clone: Tb2Shape; override;
      function GetChildCount: Int32; override;
      function TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean; override;
      function RayCast(var output: Tb2RayCastOutput; const input: Tb2RayCastInput;
         const transform: Tb2Transform; childIndex: Int32): Boolean; override;
      procedure ComputeAABB(var aabb: Tb2AABB; const xf: Tb2Transform; childIndex: Int32); override;
      procedure ComputeMass(var massData: Tb2MassData; density: Float); override;
      function ComputeSubmergedArea(const normal: TVector2; offset: Float;
         const xf: Tb2Transform; var c: TVector2): Float; override;
   end;

   /// A loop shape is a free form sequence of line segments that form a circular list.
   /// The loop may cross upon itself, but this is not recommended for smooth collision.
   /// The loop has double sided collision, so you can use inside and outside collision.
   /// Therefore, you may use any winding order.
   Tb2LoopShape = class(Tb2Shape)
   public
      m_vertices: TVectorArray;
      m_count: Int32;

      constructor Create;

      procedure SetVertices(pv: PVector2; count: Int32);
      procedure GetChildEdge(edge: Tb2EdgeShape; index: Int32);
      function Clone: Tb2Shape; override;
      function GetChildCount: Int32; override;
      function TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean; override;
      function RayCast(var output: Tb2RayCastOutput; const input: Tb2RayCastInput;
         const transform: Tb2Transform; childIndex: Int32): Boolean; override;
      procedure ComputeAABB(var aabb: Tb2AABB; const xf: Tb2Transform; childIndex: Int32); override;
      procedure ComputeMass(var massData: Tb2MassData; density: Float); override;
      function ComputeSubmergedArea(const normal: TVector2; offset: Float;
         const xf: Tb2Transform; var c: TVector2): Float; override;
   end;

   ////////////////////////////////////////////////////////////

   /// Distance joint definition. This requires defining an
   /// anchor point on both bodies and the non-zero length of the
   /// distance joint. The definition uses local anchor points
   /// so that the initial configuration can violate the constraint
   /// slightly. This helps when saving and loading a game.
   /// @warning Do not use a zero or short length.
   Tb2DistanceJointDef = class(Tb2JointDef)
   public
      localAnchorA: TVector2; /// The local anchor point relative to bodyA's origin.
      localAnchorB: TVector2; /// The local anchor point relative to bodyB's origin.

      length : Float; /// The natural length between the anchor points.
      frequencyHz: Float; /// The mass-spring-damper frequency in Hertz.
      dampingRatio: Float; /// The damping ratio. 0 = no damping, 1 = critical damping.

      constructor Create;
      procedure Initialize(bodyA, bodyB: Tb2Body; const anchorA, anchorB: TVector2);
   end;

   /// A distance joint constrains two points on two bodies
   /// to remain at a fixed distance from each other. You can view
   /// this as a massless, rigid rod.
   Tb2DistanceJoint = class(Tb2Joint)
   protected
      m_localAnchor1, m_localAnchor2, m_u: TVector2;
      m_frequencyHz, m_dampingRatio: Float;
      m_gamma, m_bias, m_impulse,
      m_mass,
      m_length: Float;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;
   public
      constructor Create(def: Tb2DistanceJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

      /// Manipulating the length can lead to non-physical behavior when the frequency is zero.
      property Length: Float read m_length write m_length;
      property Frequency: Float read m_frequencyHz write m_frequencyHz;
      property DampingRatio: Float read m_dampingRatio write m_dampingRatio;
   end;

   /// Prismatic joint definition. This requires defining a line of
   /// motion using an axis and an anchor point. The definition uses local
   /// anchor points and a local axis so that the initial configuration
   /// can violate the constraint slightly. The joint translation is zero
   /// when the local anchor points coincide in world space. Using local
   /// anchors and a local axis helps when saving and loading a game.
   /// @warning at least one body should by dynamic with a non-fixed rotation.
   Tb2PrismaticJointDef = class(Tb2JointDef)
   public
      localAnchorA: TVector2;
      localAnchorB: TVector2;

      localAxis1: TVector2; /// The local translation axis in bodyA.
      referenceAngle: Float; /// The constrained angle between the bodies: body2_angle - body1_angle.

      enableLimit: Boolean; /// Enable/disable the joint limit.

      lowerTranslation: Float; /// The lower translation limit, usually in meters.
      upperTranslation: Float; /// The upper translation limit, usually in meters.

      enableMotor: Boolean; /// Enable/disable the joint motor.
      maxMotorForce: Float; /// The maximum motor torque, usually in N-m.
      motorSpeed: Float; /// The desired motor speed in radians per second.

      constructor Create;
      procedure Initialize(bodyA, bodyB: Tb2Body; const anchor, axis: TVector2); // world anchor and world axis
   end;

   /// A prismatic joint. This joint provides one degree of freedom: translation
   /// along an axis fixed in bodyA. Relative rotation is prevented. You can
   /// use a joint limit to restrict the range of motion and a joint motor to
   /// drive the motion or to model joint friction.
   Tb2PrismaticJoint = class(Tb2Joint)
   protected
      m_localAnchor1, m_localAnchor2, m_localXAxis1, m_localYAxis1: TVector2;
      m_refAngle: Float;

      m_axis, m_perp: TVector2;
      m_s1, m_s2,
      m_a1, m_a2: Float;

      m_K: TMatrix33;
      m_impulse: TVector3;

      m_motorMass,			// effective mass for motor/limit translational constraint.
      m_motorImpulse: Float;

      m_lowerTranslation, m_upperTranslation: Float;
      m_maxMotorForce, m_motorSpeed: Float;

      m_enableLimit: Boolean;
      m_enableMotor: Boolean;

      m_limitState: Tb2LimitState;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;
   public
      constructor Create(def: Tb2PrismaticJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

      function GetJointTranslation: Float; /// Get the current joint translation, usually in meters.
	    procedure EnableLimit(flag: Boolean); /// Enable/disable the joint limit.
	    procedure EnableMotor(flag: Boolean); /// Enable/disable the joint motor.
      function GetJointSpeed: Float; /// Get the current joint translation speed, usually in meters per second.
      procedure SetLimits(lower, upper: Float); /// Set the joint limits, usually in meters.
      procedure SetMotorSpeed(speed: Float); /// Set the motor speed, usually in meters per second.
	    procedure SetMaxMotorForce(force: Float); /// Set the maximum motor force, usually in N.

      property GetMotorSpeed: Float read m_motorSpeed; // usually in meters per second.
      property GetMotorForce: Float read m_motorImpulse; // usually in N.
      property IsLimitEnabled: Boolean read m_enableLimit;
      property GetLowerLimit: Float read m_lowerTranslation;
      property GetUpperLimit: Float read m_upperTranslation;
      property IsMotorEnabled: Boolean read m_enableMotor;
      property GetMaxMotorForce: Float read m_maxMotorForce;
   end;

   /// Mouse joint definition. This requires a world target point, tuning parameters, and the time step.
   Tb2MouseJointDef = class(Tb2JointDef)
   public
      /// The initial world target point. This is assumed to coincide with the body anchor initially.
      target: TVector2;

      /// The maximum constraint force that can be exerted
      /// to move the candidate body. Usually you will express
      /// as some multiple of the weight (multiplier * mass * gravity).
      maxForce: Float;

      frequencyHz: Float; /// The response speed.
      dampingRatio: Float; /// The damping ratio. 0 = no damping, 1 = critical damping.

      constructor Create;
   end;

   /// A mouse joint is used to make a point on a body track a
   /// specified world point. This a soft constraint with a maximum
   /// force. This allows the constraint to stretch and without
   /// applying huge forces.
   /// NOTE: this joint is not documented in the manual because it was
   /// developed to be used in the testbed. If you want to learn how to
   /// use the mouse joint, look at the testbed.
   Tb2MouseJoint = class(Tb2Joint)
   protected
      m_localAnchor, m_target, m_impulse: TVector2;
      m_mass: TMatrix22; // effective mass for point-to-point constraint.
      m_C: TVector2; // position error
      m_maxForce: Float;

      m_frequencyHz,
      m_dampingRatio,
      m_beta, // bias factor
      m_gamma: Float; // softness

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;

   public
      constructor Create(def: Tb2MouseJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

	    procedure SetTarget(const target: TVector2); /// Use this to update the target point.
      function GetTarget: TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      property MaxForce: Float read m_maxForce write m_maxForce;
      property Frequency: Float read m_frequencyHz write m_frequencyHz;
      property DampingRatio: Float read m_dampingRatio write m_dampingRatio;
   end;

   /// Pulley joint definition. This requires two ground anchors,
   /// two dynamic body anchor points, max lengths for each side,
   /// and a pulley ratio.
   Tb2PulleyJointDef = class(Tb2JointDef)
   public
      groundAnchorA: TVector2; /// The first ground anchor in world coordinates. This point never moves.
      groundAnchorB: TVector2; /// The second ground anchor in world coordinates. This point never moves.
      localAnchorA: TVector2; /// The local anchor point relative to bodyA's origin.
      localAnchorB: TVector2; /// The local anchor point relative to bodyB's origin.

      lengthA: Float; /// The a reference length for the segment attached to bodyA.
      maxLengthA: Float; /// The maximum length of the segment attached to bodyA.

      lengthB: Float; /// The a reference length for the segment attached to bodyB.
      maxLengthB: Float; /// The maximum length of the segment attached to bodyB.

      ratio: Float; /// The pulley ratio, used to simulate a block-and-tackle.

      constructor Create;
      /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
      procedure Initialize(bodyA, bodyB: Tb2Body; const groundAnchorA, groundAnchorB,
        anchorA, anchorB: TVector2; ratio: Float);
   end;

   /// The pulley joint is connected to two bodies and two fixed ground points.
   /// The pulley supports a ratio such that:
   /// lengthA + ratio * lengthB <= constant
   /// Yes, the force transmitted is scaled by the ratio.
   /// The pulley also enforces a maximum length limit on both sides. This is
   /// useful to prevent one side of the pulley hitting the top.
   Tb2PulleyJoint = class(Tb2Joint)
   protected
      m_groundAnchor1, m_groundAnchor2, m_localAnchor1, m_localAnchor2: TVector2;
      m_u1, m_u2: TVector2;

      m_constant, m_ratio: Float;
      m_maxLength1, m_maxLength2: Float;

      m_pulleyMass, m_limitMass1, m_limitMass2: Float; // Effective masses

      // Impulses for accumulation/warm starting.
      m_impulse, m_limitImpulse1, m_limitImpulse2: Float;

      m_state, m_limitState1, m_limitState2: Tb2LimitState;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;
   public
      constructor Create(def: Tb2PulleyJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

      function GetLength1: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function GetLength2: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      function GetGroundAnchorA: TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
      function GetGroundAnchorB: TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      property GetRatio: Float read m_ratio;
   end;

   /// Revolute joint definition. This requires defining an
   /// anchor point where the bodies are joined. The definition
   /// uses local anchor points so that the initial configuration
   /// can violate the constraint slightly. You also need to
   /// specify the initial relative angle for joint limits. This
   /// helps when saving and loading a game.
   /// The local anchor points are measured from the body's origin
   /// rather than the center of mass because:
   /// 1. you might not know where the center of mass will be.
   /// 2. if you add/remove shapes from a body and recompute the mass,
   ///    the joints will be broken.
   Tb2RevoluteJointDef = class(Tb2JointDef)
   public
      localAnchorA: TVector2; /// The local anchor point relative to bodyA's origin.
      localAnchorB: TVector2; /// The local anchor point relative to bodyB's origin.
      referenceAngle: Float; /// The bodyB angle minus bodyA angle in the reference state (radians).

      enableLimit: Boolean; /// A flag to enable joint limits.
      lowerAngle, upperAngle: Float; /// The lower(upper) angle for the joint limit (radians).

      enableMotor: Boolean; /// A flag to enable the joint motor.
      motorOnBodyB: Boolean; /// Only apply motor to bodyB
      motorSpeed: Float; /// The desired motor speed. Usually in radians per second.
      maxMotorTorque: Float; /// The maximum motor torque used to achieve the desired motor speed. Usually in N-m.

      constructor Create;
    	/// Initialize the bodies, anchors, and reference angle using the world anchor.
    	procedure Initialize(bodyA, bodyB: Tb2Body; const anchor: TVector2);
   end;

   /// A revolute joint constrains two bodies to share a common point while they
   /// are free to rotate about the point. The relative rotation about the shared
   /// point is the joint angle. You can limit the relative rotation with
   /// a joint limit that specifies a lower and upper angle. You can use a motor
   /// to drive the relative rotation about the shared point. A maximum motor torque
   /// is provided so that infinite forces are not generated.
   Tb2RevoluteJoint = class(Tb2Joint)
   protected
      m_localAnchor1, m_localAnchor2: TVector2; // relative

	    m_impulse: TVector3;
	    m_motorImpulse: Float;

	    m_mass: TMatrix33;			// effective mass for point-to-point constraint.
	    m_motorMass: Float;	// effective mass for motor/limit angular constraint.

      m_enableMotor: Boolean;
      m_motorOnBodyB: Boolean; /// Only apply motor to bodyB
      m_maxMotorTorque, m_motorSpeed: Float;

      m_enableLimit: Boolean;
      m_referenceAngle, m_lowerAngle, m_upperAngle: Float;

      m_limitState: Tb2LimitState;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;

   public
      constructor Create(def: Tb2RevoluteJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

      function GetJointAngle: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

      /// Get the current joint angle speed in radians per second.
      function GetJointSpeed: Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

	    procedure EnableLimit(flag: Boolean); /// Enable/disable the joint limit.
	    procedure SetLimits(lower, upper: Float); /// Set the joint limits in radians.
	    procedure EnableMotor(flag: Boolean); /// Enable/disable the joint motor.
	    procedure SetMotorSpeed(speed: Float); /// Set the motor speed in radians per second.
	    procedure SetMaxMotorTorque(torque: Float);  /// Set the maximum motor torque, usually in N-m.

      property IsLimitEnabled: Boolean read m_enableLimit;
      property IsMotorEnabled: Boolean read m_enableMotor;
      property MotorOnBodyB: Boolean read m_motorOnBodyB write m_motorOnBodyB;
      property GetLowerLimit: Float read m_lowerAngle;
      property GetUpperLimit: Float read m_upperAngle;
      property GetMotorSpeed: Float read m_motorSpeed;
      property GetMotorTorque: Float read m_motorImpulse;
      property GetMaxMotorTorque: Float read m_maxMotorTorque;
   end;

   /// Gear joint definition. This definition requires two existing
   /// revolute or prismatic joints (any combination will work).
   /// The provided joints must attach a dynamic body to a static body.
   Tb2GearJointDef = class(Tb2JointDef)
   public
      joint1: Tb2Joint; /// The first revolute/prismatic joint attached to the gear joint.
      joint2: Tb2Joint; /// The second revolute/prismatic joint attached to the gear joint.
      ratio: Float; /// The gear ratio.

      constructor Create;
   end;

   /// A gear joint is used to connect two joints together. Either joint
   /// can be a revolute or prismatic joint. You specify a gear ratio
   /// to bind the motions together:
   /// coordinate1 + ratio * coordinate2 = constant
   /// The ratio can be negative or positive. If one joint is a revolute joint
   /// and the other joint is a prismatic joint, then the ratio will have units
   /// of length or units of 1/length.
   /// @warning The revolute and prismatic joints must be attached to
   /// fixed bodies (which must be bodyA on those joints).
   Tb2GearJoint = class(Tb2Joint)
   protected
      m_ground1, m_ground2: Tb2Body;

      // One of these is nil.
      m_revolute1: Tb2RevoluteJoint;
      m_prismatic1: Tb2PrismaticJoint;

      // One of these is nil.
      m_revolute2: Tb2RevoluteJoint;
      m_prismatic2: Tb2PrismaticJoint;

      m_groundAnchor1, m_groundAnchor2 :TVector2;
      m_localAnchor1, m_localAnchor2: TVector2;

      m_J: Tb2Jacobian;

      m_constant, m_ratio: Float;
      m_mass: Float; // Effective mass
      m_impulse: Float; // Impulse for accumulation/warm starting.

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;

   public
      constructor Create(def: Tb2GearJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

      property Ratio: Float read m_ratio write m_ratio;
   end;

   /// Friction joint definition.
   Tb2FrictionJointDef = class(Tb2JointDef)
   public
      localAnchorA: TVector2; /// The local anchor point relative to bodyA's origin.
      localAnchorB: TVector2; /// The local anchor point relative to bodyB's origin.
      maxForce: Float; /// The maximum friction force in N.
      maxTorque: Float; /// The maximum friction torque in N-m.

      constructor Create;
      procedure Initialize(bodyA, bodyB: Tb2Body; const anchor: TVector2); /// Initialize the bodies.
   end;

   /// Friction joint. This is used for top-down friction.
   /// It provides 2D translational friction and angular friction.
   Tb2FrictionJoint = class(Tb2Joint)
   protected
      m_localAnchorA,
      m_localAnchorB: TVector2;

      m_linearMass: TMatrix22;
      m_angularMass: Float;

      m_linearImpulse: TVector2;
      m_angularImpulse: Float;

      m_maxForce: Float;
      m_maxTorque: Float;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;
   public
      constructor Create(def: Tb2FrictionJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

      property MaxForce: Float read m_maxForce write m_maxForce;
      property MaxTorque: Float read m_maxTorque write m_maxTorque;
   end;

   /// Line joint definition. This requires defining a line of
   /// motion using an axis and an anchor point. The definition uses local
   /// anchor points and a local axis so that the initial configuration
   /// can violate the constraint slightly. The joint translation is zero
   /// when the local anchor points coincide in world space. Using local
   /// anchors and a local axis helps when saving and loading a game.
   Tb2LineJointDef = class(Tb2JointDef)
   public
      localAnchorA: TVector2; /// The local anchor point relative to body1's origin.
      localAnchorB: TVector2; /// The local anchor point relative to body2's origin.
      localAxisA: TVector2; /// The local translation axis in body1.
      enableLimit: Boolean; /// Enable/disable the joint limit.

      lowerTranslation: Float; /// The lower translation limit, usually in meters.
      upperTranslation: Float; /// The upper translation limit, usually in meters.

      enableMotor: Boolean; /// Enable/disable the joint motor.
      maxMotorForce: Float; /// The maximum motor torque, usually in N-m.

      motorSpeed: Float; /// The desired motor speed in radians per second.

      constructor Create;
      /// Initialize the bodies, anchors, axis, and reference angle using the world
      /// anchor and world axis.
      procedure Initialize(bodyA, bodyB: Tb2Body; const anchor, axis: TVector2);
   end;

   /// A line joint. This joint provides two degrees of freedom: translation
   /// along an axis fixed in body1 and rotation in the plane. You can use a
   /// joint limit to restrict the range of motion and a joint motor to drive
   /// the motion or to model joint friction.
   Tb2LineJoint = class(Tb2Joint)
   protected
      m_localAnchor1,
      m_localAnchor2,
      m_localXAxis1,
      m_localYAxis1: TVector2;

      m_axis, m_perp:  TVector2;
      m_s1, m_s2,
      m_a1, m_a2: Float;

      m_K: TMatrix22;
      m_impulse: TVector2;

      m_motorMass: Float;			// effective mass for motor/limit translational constraint.
      m_motorImpulse: Float;

      m_lowerTranslation,
      m_upperTranslation,
      m_maxMotorForce,
      m_motorSpeed: Float;

      m_enableLimit: Boolean;
      m_enableMotor: Boolean;
      m_limitState: Tb2LimitState;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;

   public
      constructor Create(def: Tb2LineJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;

      /// Get the current joint translation, usually in meters.
      function GetJointTranslation: Float;

      /// Get the current joint translation speed, usually in meters per second.
      function GetJointSpeed: Float;

      procedure EnableLimit(flag: Boolean); /// Enable/disable the joint limit.
      procedure SetLimits(lower, upper: Float); /// Set the joint limits, usually in meters.

      procedure EnableMotor(flag: Boolean); /// Enable/disable the joint motor.
      procedure SetMotorSpeed(speed: Float); /// Set the motor speed, usually in meters per second.
      procedure SetMaxMotorForce(force: Float); /// Set the maximum motor force, usually in N.

      property IsLimitEnabled: Boolean read m_enableLimit; /// Is the joint limit enabled?
      property GetLowerLimit: Float read m_lowerTranslation; /// Get the lower joint limit, usually in meters.
      property GetUpperLimit: Float read m_upperTranslation; /// Get the upper joint limit, usually in meters.
      property IsMotorEnabled: Boolean read m_enableMotor; /// Is the joint motor enabled?
      property GetMotorSpeed: Float read m_motorSpeed; /// Get the motor speed, usually in meters per second.
      property GetMaxMotorForce: Float read m_maxMotorForce;
      property GetMotorForce: Float read m_motorImpulse; /// Get the current motor force, usually in N.
   end;

   /// Weld joint definition. You need to specify local anchor points
   /// where they are attached and the relative body angle. The position
   /// of the anchor points is important for computing the reaction torque.
   Tb2WeldJointDef = class(Tb2JointDef)
   public
      /// The local anchor point relative to body1's origin.
      localAnchorA: TVector2;

      /// The local anchor point relative to body2's origin.
      localAnchorB: TVector2;

      /// The body2 angle minus body1 angle in the reference state (radians).
      referenceAngle: Float;

      constructor Create;

      /// Initialize the bodies, anchors, and reference angle using a world
      /// anchor point.
      procedure Initialize(bodyA, bodyB: Tb2Body; const anchor: TVector2);
   end;

   /// A weld joint essentially glues two bodies together. A weld joint may
   /// distort somewhat because the island constraint solver is approximate.
   Tb2WeldJoint = class(Tb2Joint)
   protected
      m_localAnchorA, m_localAnchorB: TVector2;
      m_referenceAngle: Float;

      m_impulse: TVector3;
      m_mass: TMatrix33;

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;
   public
      constructor Create(def: Tb2WeldJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;
   end;

   /// FixedJoint: Attaches two bodies rigidly together
   Tb2FixedJointDef = class(Tb2JointDef)
   public
      constructor Create;
	    procedure Initialize(bodyA, bodyB: Tb2Body); /// Initialize the bodies.
   end;

   /// A fixed joint constrains all degrees of freedom between two bodies
   /// Author: Jorrit Rouwe
   /// See: www.jrouwe.nl/fixedjoint/ for more info
   Tb2FixedJoint = class(Tb2Joint)
   private
	    procedure CalculateMC; // Get effective constraint mass
   protected
      // Configured state for bodies
      m_dp: TVector2;		//< Distance between body->GetTransform().position between the two bodies at rest in the reference frame of bodyA
      m_a: Float;		//< Angle between the bodies at rest
      m_R0: TMatrix22;		//< Rotation matrix of m_a

      // State for solving
      m_inv_dt: Float;	//< Stored 1/dt
      m_d: TVector2;			//< Distance between center of masses for this time step (when the shapes of the bodies change, their local centers can change so we derive this from m_dp every frame)
      m_a1: Float;			//< Stored angle of body 1 (a1) to determine if it changed
      m_c, m_s: Float;		//< cos(a1) and sin(a1)
      m_Ax, m_Ay: Float;	//< A = d/dt (R(a1) d)
      m_mc: array[0..2, 0..2] of Float;	//< Effective constraint mass

      // State after solving
      m_lambda: array[0..2] of Float;	//< Accumulated lambdas for warm starting and returning constraint force

      procedure InitVelocityConstraints(const step: Tb2TimeStep); override;
      procedure SolveVelocityConstraints(const step: Tb2TimeStep); override;
      function SolvePositionConstraints(baumgarte: Float): Boolean; override;
   public
      constructor Create(def: Tb2FixedJointDef);

      function GetAnchorA: TVector2; override;
      function GetAnchorB: TVector2; override;

      function GetReactionForce(inv_dt: Float): TVector2; override;
      function GetReactionTorque(inv_dt: Float): Float; override;
   end;

procedure b2GetPointStates(var state1, state2: Tb2PointStateArray;
   const manifold1, manifold2: Tb2Manifold);

// Evaluate functions for different contacts
procedure b2CollideCircles(contact: Pb2Contact; var manifold: Tb2Manifold; A, B: TObject;
   const xfA, xfB: Tb2Transform; ABfixture: Boolean);
procedure b2CollidePolygonAndCircle(contact: Pb2Contact; var manifold: Tb2Manifold; A, B: TObject;
   const xfA, xfB: Tb2Transform; ABfixture: Boolean);
procedure b2CollideEdgeAndCircle(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
procedure b2CollideEdgeAndPolygon(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
procedure b2CollideLoopAndCircle(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
procedure b2CollideLoopAndPolygon(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
procedure b2CollidePolygons(contact: Pb2Contact; var manifold: Tb2Manifold; A, B: TObject;
   const xfA, xfB: Tb2Transform; ABfixture: Boolean);

procedure b2Distance(var output: Tb2DistanceOutput;	var cache: Tb2SimplexCache;
   const input: Tb2DistanceInput);

function b2TestOverlap(const a, b: Tb2AABB): Boolean; overload;
function b2TestOverlap(shapeA, shapeB: Tb2Shape; indexA, indexB: Int32;
   const xfA, xfB: Tb2Transform): Boolean; overload;

function b2TimeOfImpact(var output: Tb2TOIOutput; const input: Tb2TOIInput): Float;

{$IFNDEF OP_OVERLOAD}
// Methods for records

/// Tb2Contact
function GetManifold(const contact: Tb2Contact): Pb2Manifold; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure GetWorldManifold(const contact: Tb2Contact; var worldManifold: Tb2WorldManifold); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure FlagForFiltering(var contact: Tb2Contact); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function IsTouching(const contact: Tb2Contact): Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetEnabled(var contact: Tb2Contact; flag: Boolean); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function IsEnabled(const contact: Tb2Contact): Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF} /// Has this contact been disabled?
procedure Update(var contact: Tb2Contact; listener: Tb2ContactListener);

/// Tb2DistanceProxy
procedure SetShape(var dp: Tb2DistanceProxy; shape: Tb2Shape; index: Int32);
function GetSupport(const dp: Tb2DistanceProxy; const d: TVector2): Int32;
function GetSupportVertex(const dp: Tb2DistanceProxy; const d: TVector2): PVector2;

/// Tb2WorldManifold
procedure Initialize(var worldManifold: Tb2WorldManifold;
   const manifold: Tb2Manifold; const xfA, xfB: Tb2Transform; radiusA, radiusB: Float);

/// Tb2AABB
function IsValid(const AABB: Tb2AABB): Boolean; overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function GetCenter(const AABB: Tb2AABB): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function GetExtents(const AABB: Tb2AABB): TVector2; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function GetPerimeter(const AABB: Tb2AABB): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure Combine(var AABB: Tb2AABB; const _aabb: Tb2AABB); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure Combine(var AABB: Tb2AABB; const aabb1, aabb2: Tb2AABB); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Contains(const AABB, _aabb: Tb2AABB): Boolean;
function RayCast(const AABB: Tb2AABB; var output: Tb2RayCastOutput; const input: Tb2RayCastInput): Boolean;

/// Tb2Jacobian
procedure SetZero(var jb: Tb2Jacobian); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
procedure SetValue(var jb: Tb2Jacobian; const x1, x2: TVector2; a1, a2: Float); overload; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
function Compute(var jb: Tb2Jacobian; const x1, x2: TVector2; a1, a2: Float): Float; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}

/// Tb2DynamicTreeNode
function IsLeaf(const node: Tb2DynamicTreeNode): Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
{$ENDIF}

/////////////////// Color functions //////
function MakeColor(r, g, b: Single; a: Single = 1.0): RGBA;

var
   b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters: Int32;
   b2_toiCalls, b2_toiIters, b2_toiMaxIters,
   b2_toiRootIters, b2_toiMaxRootIters: Int32;

const
   b2_nullNode = -1;

implementation
var
   b2_defaultFilter: Tb2ContactFilter;
   b2_defaultListener: Tb2ContactListener;

   {$IFDEF COMPUTE_PHYSICSTIME}
   vCounterFrequency: Int64;
   {$ENDIF}
const
   e_nullProxy = -1;

   // Tb2World
   e_world_newFixture	= 1;
   e_world_locked	= 2;
   e_world_clearForces = 4;

   // Tb2Contact.m_flags
   e_contact_islandFlag	 = $1; // Used when crawling contact graph when forming islands.
   e_contact_touchingFlag	= $2; // Set when the shapes are touching.
   e_contact_enabledFlag = $4; // This contact can be disabled (by user)
   e_contact_filterFlag	= $8; // This contact needs filtering because a fixture filter was changed.
	 e_contact_bulletHitFlag = $10; // This bullet contact had a TOI event
   e_contact_toiFlag = $20; // This contact has a valid TOI in m_toi

   // Tb2Body.m_flags
   e_body_islandFlag = $1;
   e_body_awakeFlag	= $2;
   e_body_autoSleepFlag	= $4;
   e_body_bulletFlag = $8;
   e_body_fixedRotationFlag	= $10;
   e_body_activeFlag	= $20;
   e_body_toiFlag = $40;
   e_body_ignoreCollideFlag = $80;

   // Tb2ContactFeature
   e_contact_feature_vertex = 0;
   e_contact_feature_face = 1;

function MakeColor(r, g, b: Single; a: Single = 1.0): RGBA;
begin
   Result[0] := r;
   Result[1] := g;
   Result[2] := b;
   Result[3] := a;
end;

//////////// Implements <b2Contact.cpp> InitializeRegisters and AddType
type
   TContactCreateRecord = record
      //ContactType: Tb2ContactType;
      EvaluateProc: Tb2ContactEvaluateProc;
      Primary: Boolean;
   end;

const
   // e_circleShape, e_edgeShape, e_polygonShape, e_loopShape
   ContactCreateRecords: array[e_circleShape..e_loopShape,
      e_circleShape..e_loopShape] of TContactCreateRecord = (
      // e_circleShape with
      ((EvaluateProc: b2CollideCircles; Primary: True), // with e_circleShape
       (EvaluateProc: b2CollideEdgeAndCircle; Primary: False), // with e_edgeShape
       (EvaluateProc: b2CollidePolygonAndCircle; Primary: False), // with e_polygonShape
       (EvaluateProc: b2CollideLoopAndCircle; Primary: False)), // with e_loopShape

      // e_edgeShape with
      ((EvaluateProc: b2CollideEdgeAndCircle; Primary: True), // with e_circleShape
       (EvaluateProc: nil; Primary: True), // with e_edgeShape
       (EvaluateProc: b2CollideEdgeAndPolygon; Primary: True), // with e_polygonShape
       (EvaluateProc: nil; Primary: True)), // with e_loopShape

      // e_polygonShape with
      ((EvaluateProc: b2CollidePolygonAndCircle; Primary: True), // with e_circleShape
       (EvaluateProc: b2CollideEdgeAndPolygon; Primary: False), // with e_edgeShape
       (EvaluateProc: b2CollidePolygons; Primary: True), // with e_polygonShape
       (EvaluateProc: b2CollideLoopAndPolygon; Primary: False)), // with e_loopShape

      // e_loopShape with
      ((EvaluateProc: b2CollideLoopAndCircle; Primary: True), // with e_circleShape
       (EvaluateProc: nil; Primary: False), // with e_edgeShape
       (EvaluateProc: b2CollideLoopAndPolygon; Primary: True), // with e_polygonShape
       (EvaluateProc: nil; Primary: False)) // with e_loopShape
       );

function NewContact(fixtureA, fixtureB: Tb2Fixture; indexA, indexB: Int32): Pb2Contact;
begin
   New(Result);
   FillChar(Result^, SizeOf(Tb2Contact), 0);
   with Result^ do
   begin
      m_flags := e_contact_enabledFlag;
      with ContactCreateRecords[fixtureA.m_shape.m_type][fixtureB.m_shape.m_type] do
      begin
         //m_type := ContactType;
         m_evaluateProc := EvaluateProc;
         if Primary then
         begin
            m_fixtureA := fixtureA;
            m_fixtureB := fixtureB;
            m_indexA := indexA;
            m_indexB := indexB;
         end
         else
         begin
            m_fixtureA := fixtureB;
            m_fixtureB := fixtureA;
            m_indexA := indexB;
            m_indexB := indexA;
         end;
      end;
   end;
end;

procedure FreeContact(pc: Pb2Contact);
begin
   with pc^ do
      if m_manifold.pointCount > 0 then
      begin
         m_fixtureA.m_body.SetAwake(True);
         m_fixtureB.m_body.SetAwake(True);
      end;
   Dispose(pc);
end;

{$IFNDEF OP_OVERLOAD}
// Record methods

/// Tb2Contact
function GetManifold(const contact: Tb2Contact): Pb2Manifold;
begin
   Result := @contact.m_manifold;
end;

procedure GetWorldManifold(const contact: Tb2Contact; var worldManifold: Tb2WorldManifold);
begin
   with contact do
      {$IFDEF OP_OVERLOAD}
      worldManifold.Initialize(m_manifold, m_fixtureA.m_body.m_xf,
         m_fixtureB.m_body.m_xf, m_fixtureA.m_shape.m_radius, m_fixtureB.m_shape.m_radius);
      {$ELSE}
      Initialize(worldManifold, m_manifold, m_fixtureA.m_body.m_xf,
         m_fixtureB.m_body.m_xf, m_fixtureA.m_shape.m_radius, m_fixtureB.m_shape.m_radius);
      {$ENDIF}
end;

procedure FlagForFiltering(var contact: Tb2Contact);
begin
   with contact do
      m_flags := m_flags or e_contact_filterFlag;
end;

function IsTouching(const contact: Tb2Contact): Boolean;
begin
   Result := (contact.m_flags and e_contact_touchingFlag) = e_contact_touchingFlag;
end;

procedure SetEnabled(var contact: Tb2Contact; flag: Boolean);
begin
   with contact do
      if flag then
         m_flags := m_flags or e_contact_enabledFlag
      else
         m_flags := m_flags and (not e_contact_enabledFlag);
end;

function IsEnabled(const contact: Tb2Contact): Boolean;
begin
   Result := (contact.m_flags and e_contact_enabledFlag) = e_contact_enabledFlag;
end;

procedure Update(var contact: Tb2Contact; listener: Tb2ContactListener);
var
   i, j: Integer;
   oldManifold: Tb2Manifold;
   touching, wasTouching, sensor: Boolean;
   bodyA, bodyB: Tb2Body;
   mp1, mp2: Pb2ManifoldPoint;
   id2key: UInt32;
   found: Boolean;
begin
   with contact do
   begin
      oldManifold := m_manifold;
      m_flags := m_flags or e_contact_enabledFlag; // Re-enable this contact.

      //touching := False;
      wasTouching := (m_flags and e_contact_touchingFlag) = e_contact_touchingFlag;

      sensor := m_fixtureA.IsSensor or m_fixtureB.IsSensor;

      bodyA := m_fixtureA.m_body;
      bodyB := m_fixtureB.m_body;

      // Is this contact a sensor?
      if sensor then
      begin
         touching := b2TestOverlap(m_fixtureA.m_shape, m_fixtureB.m_shape,
            m_indexA, m_indexB, bodyA.m_xf, bodyB.m_xf);

         // Sensors don't generate manifolds.
         m_manifold.pointCount := 0;
      end
      else
      begin
         m_evaluateProc(@contact, m_manifold, m_fixtureA, m_fixtureB, bodyA.m_xf, bodyB.m_xf, True);
         touching := m_manifold.pointCount > 0;

         // Match old contact ids to new contact ids and copy the
         // stored impulses to warm start the solver.
         for i := 0 to m_manifold.pointCount - 1 do
         begin
            mp2 := @m_manifold.points[i];
            mp2^.normalImpulse := 0.0;
            mp2^.tangentImpulse := 0.0;
            id2key := mp2^.id.key;
            found := False;

            for j := 0 to oldManifold.pointCount - 1 do
            begin
               mp1 := @oldManifold.points[j];

               if mp1^.id.key = id2key then
               begin
                  mp2^.normalImpulse := mp1^.normalImpulse;
                  mp2^.tangentImpulse := mp1^.tangentImpulse;
                  found := True;
                  Break;
               end;
            end;

            if not found then
            begin
               mp2^.normalImpulse := 0.0;
               mp2^.tangentImpulse := 0.0;
            end;
         end;

         if touching xor wasTouching then
         begin
            bodyA.SetAwake(True);
            bodyB.SetAwake(True);
         end;
      end;

      if touching then
         m_flags := m_flags or e_contact_touchingFlag
      else
         m_flags := m_flags and (not e_contact_touchingFlag);

      if Assigned(listener) then
      begin
         if (not wasTouching) and touching then
            listener.BeginContact(contact);

         if wasTouching and (not touching) then
            listener.EndContact(contact);

         if (not sensor) and touching then
            listener.PreSolve(contact, oldManifold);
      end;
   end;
end;

/// Tb2WorldManifold
procedure Initialize(var worldManifold: Tb2WorldManifold;
   const manifold: Tb2Manifold; const xfA, xfB: Tb2Transform; radiusA, radiusB: Float);
var
   i: Integer;
   pointA, pointB, cA, cB, planePoint, clipPoint: TVector2;
begin
   if manifold.pointCount = 0 then
      Exit;

   with worldManifold do
      case manifold.manifoldType of
         e_manifold_circles:
            begin
               normal.x := 1.0;
               normal.y := 0.0;

               pointA := b2Mul(xfA, manifold.localPoint);
               pointB := b2Mul(xfB, manifold.points[0].localPoint);
               if b2DistanceSquared(pointA, pointB) > FLT_EPSILON * FLT_EPSILON then
               begin
                  normal := Subtract(pointB, pointA);
                  Normalize(normal);
               end;

               cA := Add(pointA, Multiply(normal, radiusA));
               cB := Subtract(pointB, Multiply(normal, radiusB));
               points[0] := b2MiddlePoint(cA, cB);
            end;
         e_manifold_faceA:
            begin
               normal := b2Mul(xfA.R, manifold.localNormal);
               planePoint := b2Mul(xfA, manifold.localPoint);

               for i := 0 to manifold.pointCount - 1 do
               begin
                  clipPoint := b2Mul(xfB, manifold.points[i].localPoint{manifold.points});
                  cA := Add(clipPoint, Multiply(normal,
                     (radiusA - b2Dot(Subtract(clipPoint, planePoint), normal))));
                  cB := Subtract(clipPoint, Multiply(normal, radiusB));
                  points[i] := b2MiddlePoint(cA, cB);
               end;
            end;
         e_manifold_faceB:
            begin
               normal := b2Mul(xfB.R, manifold.localNormal);
               planePoint := b2Mul(xfB, manifold.localPoint);

               for i := 0 to manifold.pointCount - 1 do
               begin
                  clipPoint := b2Mul(xfA, manifold.points[i].localPoint{manifold.points});
                  cB := Add(clipPoint, Multiply(normal,
                     (radiusB - b2Dot(Subtract(clipPoint, planePoint), normal))));
                  cA := Subtract(clipPoint, Multiply(normal, radiusA));
                  points[i] := b2MiddlePoint(cA, cB);
               end;

               // Ensure normal points from A to B.
               normal := Negative(normal);
            end;
      end;
end;

/// Tb2AABB
function IsValid(const AABB: Tb2AABB): Boolean;
var
   d: TVector2;
begin
   with AABB do
   begin
      d := Subtract(upperBound, lowerBound);
      Result := (d.x >= 0.0) and (d.y >= 0.0) and 
         UPhysics2DTypes.IsValid(upperBound) and UPhysics2DTypes.IsValid(lowerBound);
   end;
end;

function GetCenter(const AABB: Tb2AABB): TVector2;
begin
   with AABB do
      Result := b2MiddlePoint(lowerBound, upperBound);
end;

function GetExtents(const AABB: Tb2AABB): TVector2;
begin
   with AABB do
      Result := Multiply(Subtract(upperBound, lowerBound), 0.5);
end;

function GetPerimeter(const AABB: Tb2AABB): Float;
begin
   with AABB do
      Result := 2.0 * ((upperBound.x - lowerBound.x) + (upperBound.y - lowerBound.y));
end;

procedure Combine(var AABB: Tb2AABB; const _aabb: Tb2AABB);
begin
   with AABB do
   begin
   	  lowerBound := b2Min(lowerBound, _aabb.lowerBound);
   	  upperBound := b2Max(upperBound, _aabb.upperBound);
   end;
end;

procedure Combine(var AABB: Tb2AABB; const aabb1, aabb2: Tb2AABB);
begin
   with AABB do
   begin
      lowerBound := b2Min(aabb1.lowerBound, aabb2.lowerBound);
      upperBound := b2Max(aabb1.upperBound, aabb2.upperBound);
   end;
end;

function Contains(const AABB, _aabb: Tb2AABB): Boolean;
begin
   with AABB do
   begin
      Result := True;
      Result := Result and (lowerBound.x <= _aabb.lowerBound.x);
      Result := Result and (lowerBound.y <= _aabb.lowerBound.y);
      Result := Result and (_aabb.upperBound.x <= upperBound.x);
      Result := Result and (_aabb.upperBound.y <= upperBound.y);
   end;
end;

function RayCast(const AABB: Tb2AABB; var output: Tb2RayCastOutput;
   const input: Tb2RayCastInput): Boolean;
var
   i: Integer;
   tmin, tmax, inv_d, t1, t2, s: Float;
   p, d, absD, normal: TVector2;
begin
   with AABB do
   begin
      tmin := -FLT_MAX;
      tmax := FLT_MAX;

      p := input.p1;
      d := Subtract(input.p2, input.p1);
      absD := b2Abs(d);

      for i := 0 to 1 do
      begin
         if TVector2Arraied(absD)[i] < FLT_EPSILON then
         begin
            // Parallel.
            if (TVector2Arraied(p)[i] < TVector2Arraied(lowerBound)[i]) or
               (TVector2Arraied(upperBound)[i] < TVector2Arraied(p)[i]) then
            begin
               Result := False;
               Exit;
            end;
         end
         else
         begin
            inv_d := 1.0 / TVector2Arraied(d)[i];
            t1 := (TVector2Arraied(lowerBound)[i] - TVector2Arraied(p)[i]) * inv_d;
            t2 := (TVector2Arraied(upperBound)[i] - TVector2Arraied(p)[i]) * inv_d;

            // Sign of the normal vector.
            s := -1.0;

            if t1 > t2 then
            begin
               b2Swap(t1, t2);
               s := 1.0;
            end;

            // Push the min up
            if t1 > tmin then
            begin
               normal := b2Vec2_Zero;
               TVector2Arraied(normal)[i] := s;
               tmin := t1;
            end;

            // Pull the max down
            tmax := b2Min(tmax, t2);

            if tmin > tmax then
            begin
               Result := False;
               Exit;
            end;
         end;
      end;

      // Does the ray start inside the box?
      // Does the ray intersect beyond the max fraction?
      if (tmin < 0.0) or (input.maxFraction < tmin) then
      begin
         Result := False;
         Exit;
      end;

      // Intersection.
      output.fraction := tmin;
      output.normal := normal;
      Result := True;
   end;
end;

/// Tb2Jacobian
procedure SetZero(var jb: Tb2Jacobian);
begin
   with jb do
   begin
      linearA := b2Vec2_Zero;
      linearB := b2Vec2_Zero;
      angularA := 0.0;
      angularB := 0.0;
   end;
end;

procedure SetValue(var jb: Tb2Jacobian; const x1, x2: TVector2; a1, a2: Float);
begin
   with jb do
   begin
      linearA := x1;
      linearB := x2;
      angularA := a1;
      angularB := a2;
   end;
end;

function Compute(var jb: Tb2Jacobian; const x1, x2: TVector2; a1, a2: Float): Float;
begin
   with jb do
      Result := b2Dot(linearA, x1) + angularA * a1 + b2Dot(linearB, x2) + angularB * a2;
end;

{$ENDIF}

{ b2PolygonShape.cpp}

function ComputeCentroid(const vs: Tb2PolyVertices; count: Int32): TVector2;
const
   inv3 = 1 / 3;
var
   i: Integer;
   pRef, p1, p2, p3, e1, e2: TVector2;
   area, triangleArea: Float;
begin
   //b2Assert(count >= 2);

   area := 0.0;
   if count = 2 then
   begin
      Result := b2MiddlePoint(vs[0], vs[1]);
      Exit;
   end;

   // pRef is the reference point for forming triangles.
   // It's location doesn't change the result (except for rounding error).
   Result := b2Vec2_Zero;
   pRef := b2Vec2_Zero;

  (* // This code would put the reference point inside the polygon.
   for (int32 i = 0; i < count; ++i)
   {
     pRef += vs[i];
   }
   pRef *= 1.0f / count; *)

   for i := 0 to count - 1 do
   begin
      // Triangle vertices.
      p1 := pRef;
      p2 := vs[i];
      if i + 1 < count then
         p3 := vs[i + 1]
      else
         p3 := vs[0];   

      {$IFDEF OP_OVERLOAD}         
      e1 := p2 - p1;
      e2 := p3 - p1;
      {$ELSE}      
      e1 := Subtract(p2, p1);
      e2 := Subtract(p3, p1);
      {$ENDIF}            

      triangleArea := 0.5 * b2Cross(e1, e2);
      area := area + triangleArea;

      // Area weighted centroid    

      {$IFDEF OP_OVERLOAD}        
      Result.AddBy(triangleArea * inv3 * (p1 + p2 + p3));
      {$ELSE}
      AddBy(Result, Multiply(Add(p1, p2, p3), triangleArea * inv3));
      {$ENDIF}
   end;

   // Centroid
   //b2Assert(area > B2_FLT_EPSILON);
   {$IFDEF OP_OVERLOAD}        
   Result := Result / area;
   {$ELSE}   
   DivideBy(Result, area);
   {$ENDIF}  
end;

{ b2Distance.cpp }
{$IFDEF OP_OVERLOAD}
{ Tb2DistanceProxy }
procedure Tb2DistanceProxy.SetShape(shape: Tb2Shape; index: Int32);
begin
   case shape.m_type of
      e_circleShape:
         with Tb2CircleShape(shape) do
         begin
            m_vertices := @m_p;
            m_count := 1;
            Self.m_radius := m_radius;
         end;
      e_polygonShape:
         with Tb2PolygonShape(shape) do
         begin
            Self.m_vertices := @m_vertices[0];
            Self.m_count := m_vertexCount;
            Self.m_radius := m_radius;
         end;
      e_edgeShape:
         with Tb2EdgeShape(shape) do
         begin
            Self.m_vertices := @m_vertex1;
            Self.m_count := 2;
            Self.m_radius := m_radius;
         end;
      e_loopShape:
         with Tb2LoopShape(shape) do
         begin
            //b2Assert(0 <= index && index < loop->m_count);
            m_buffer[0] := m_vertices[index];
            if index + 1 < m_count then
               m_buffer[1] := m_vertices[index + 1]
            else
               m_buffer[1] := m_vertices[0];

            Self.m_vertices := @m_buffer[0];
            Self.m_count := 2;
            Self.m_radius := m_radius;
         end;
   end;
end;

function Tb2DistanceProxy.GetSupport(const d: TVector2): Int32;
var
   i: Integer;
   bestValue, value: Float;
   {$IFNDEF D2009UP}p: PVector2;{$ENDIF}
begin
   Result := 0;
   {$IFDEF D2009UP}
   bestValue := b2Dot(m_vertices[0], d);
   for i := 1 to m_count - 1 do
   begin
      value := b2Dot(m_vertices[i], d);
      if value > bestValue then
      begin
         Result := i;
         bestValue := value;
      end;
   end;
   {$ELSE}
   bestValue := b2Dot(m_vertices^, d);
   for i := 1 to m_count - 1 do
   begin
      p := m_vertices;
      Inc(p, i);
      value := b2Dot(p^, d);
      if value > bestValue then
      begin
         Result := i;
         bestValue := value;
      end;
   end;
   {$ENDIF}
end;

function Tb2DistanceProxy.GetSupportVertex(const d: TVector2): PVector2;
var
   i, bestIndex: Integer;
   bestValue, value: Float;
   {$IFNDEF D2009UP}p: PVector2;{$ENDIF}
begin
   bestIndex := 0;
   {$IFDEF D2009UP}
   bestValue := b2Dot(m_vertices[0], d);
   for i := 1 to m_count - 1 do
   begin
      value := b2Dot(m_vertices[i], d);
      if value > bestValue then
      begin
         bestIndex := i;
         bestValue := value;
      end;
   end;
   Result := @m_vertices[bestIndex];
   {$ELSE}
   bestValue := b2Dot(m_vertices^, d);
   for i := 1 to m_count - 1 do
   begin
      p := m_vertices;
      Inc(p, i);
      value := b2Dot(p^, d);
      if value > bestValue then
      begin
         bestIndex := i;
         bestValue := value;
      end;
   end;
   Result := m_vertices;
   Inc(Result, bestIndex);
   {$ENDIF}
end;
{$ELSE}
procedure SetShape(var dp: Tb2DistanceProxy; shape: Tb2Shape; index: Int32);
begin
   with dp do
      case shape.m_type of
         e_circleShape:
            with Tb2CircleShape(shape) do
            begin
               dp.m_vertices := @m_p;
               dp.m_count := 1;
               dp.m_radius := m_radius;
            end;
         e_polygonShape:
            with Tb2PolygonShape(shape) do
            begin
               dp.m_vertices := @m_vertices[0];
               dp.m_count := m_vertexCount;
               dp.m_radius := m_radius;
            end;
         e_edgeShape:
            with Tb2EdgeShape(shape) do
            begin
               dp.m_vertices := @m_vertex1;
               dp.m_count := 2;
               dp.m_radius := m_radius;
            end;
         e_loopShape:
            with Tb2LoopShape(shape) do
            begin
               //b2Assert(0 <= index && index < loop->m_count);
               m_buffer[0] := m_vertices[index];
               if index + 1 < m_count then
                  m_buffer[1] := m_vertices[index + 1]
               else
                  m_buffer[1] := m_vertices[0];

               dp.m_vertices := @m_buffer[0];
               dp.m_count := 2;
               dp.m_radius := m_radius;
            end;
      end;
end;

function GetSupport(const dp: Tb2DistanceProxy; const d: TVector2): Int32;
var
   i: Integer;
   bestValue, value: Float;
   {$IFNDEF D2009UP}p: PVector2;{$ENDIF}
begin
   with dp do
   begin
      Result := 0;
      {$IFDEF D2009UP}
      bestValue := b2Dot(m_vertices[0], d);
      for i := 1 to m_count - 1 do
      begin
         value := b2Dot(m_vertices[i], d);
         if value > bestValue then
         begin
            Result := i;
            bestValue := value;
         end;
      end;
      {$ELSE}
      bestValue := b2Dot(m_vertices^, d);
      for i := 1 to m_count - 1 do
      begin
         p := m_vertices;
         Inc(p, i);
         value := b2Dot(p^, d);
         if value > bestValue then
         begin
            Result := i;
            bestValue := value;
         end;
      end;
      {$ENDIF}
   end;
end;

function GetSupportVertex(const dp: Tb2DistanceProxy; const d: TVector2): PVector2;
var
   i, bestIndex: Integer;
   bestValue, value: Float;
   {$IFNDEF D2009UP}p: PVector2;{$ENDIF}
begin
   with dp do
   begin
      bestIndex := 0;
      {$IFDEF D2009UP}
      bestValue := b2Dot(m_vertices[0], d);
      for i := 1 to m_count - 1 do
      begin
         value := b2Dot(m_vertices[i], d);
         if value > bestValue then
         begin
            bestIndex := i;
            bestValue := value;
         end;
      end;
      Result := @m_vertices[bestIndex];
      {$ELSE}
      bestValue := b2Dot(m_vertices^, d);
      for i := 1 to m_count - 1 do
      begin
         p := m_vertices;
         Inc(p, i);
         value := b2Dot(p^, d);
         if value > bestValue then
         begin
            bestIndex := i;
            bestValue := value;
         end;
      end;
      Result := m_vertices;
      Inc(Result, bestIndex);
      {$ENDIF}
   end;
end;
{$ENDIF}

{ b2TimeOfImpact.cpp }
type
   Tb2SeparationFunctionType = (e_separation_points, e_separation_faceA, e_separation_faceB);
   Tb2SeparationFunction = class
   public
      m_proxyA, m_proxyB: Pb2DistanceProxy;
      m_sweepA, m_sweepB: Tb2Sweep;
      m_type: Tb2SeparationFunctionType;
      m_localPoint,
      m_axis: TVector2;

      function Initialize(const cache: Tb2SimplexCache; const proxyA,
         proxyB: Tb2DistanceProxy; const sweepA, sweepB: Tb2Sweep; const t1: Float): Float;
      function FindMinSeparation(var indexA, indexB: Int32; t: Float): Float;
      function Evaluate(indexA, indexB: Int32; t: Float): Float;
   end;

{ Tb2SeparationFunction }

function Tb2SeparationFunction.Initialize(const cache: Tb2SimplexCache;
   const proxyA, proxyB: Tb2DistanceProxy; const sweepA, sweepB: Tb2Sweep;
   const t1: Float): Float;
var
   xfA, xfB: Tb2Transform;
   localPointB1, localPointB2, localPointA1, localPointA2, normal: TVector2;
   {$IFNDEF D2009UP}pA, pB: PVector2;{$ENDIF}
begin
   m_proxyA := @proxyA;
   m_proxyB := @proxyB;
   //b2Assert(0 < count && count < 3);

   m_sweepA := sweepA;
   m_sweepB := sweepB;

   {$IFDEF OP_OVERLOAD}
   m_sweepA.GetTransform(xfA, t1);
   m_sweepB.GetTransform(xfB, t1);
   {$ELSE}
   GetTransform(m_sweepA, xfA, t1);
   GetTransform(m_sweepB, xfB, t1);
   {$ENDIF}

   if cache.count = 1 then
   begin
      m_type := e_separation_points;
      {$IFNDEF D2009UP}
      pA := proxyA.m_vertices;
      pB := proxyB.m_vertices;
      Inc(pA, cache.indexA[0]);
      Inc(pB, cache.indexB[0]);
      {$ENDIF}

      {$IFDEF OP_OVERLOAD}
         {$IFDEF D2009UP}
         m_axis := b2Mul(xfB, proxyB.m_vertices[cache.indexB[0]]) -
            b2Mul(xfA, proxyA.m_vertices[cache.indexA[0]]);
         {$ELSE}
         m_axis := b2Mul(xfB, pB^) - b2Mul(xfA, pA^);
         {$ENDIF}
         Result := m_axis.Normalize;
      {$ELSE}
         {$IFDEF D2009UP}
         m_axis := Subtract(b2Mul(xfB, proxyB.m_vertices[cache.indexB[0]]),
            b2Mul(xfA, proxyA.m_vertices[cache.indexA[0]]));
         {$ELSE}
         m_axis := Subtract(b2Mul(xfB, pB^), b2Mul(xfA, pA^));
         {$ENDIF}
         Result := Normalize(m_axis);
      {$ENDIF}
   end
   else if cache.indexA[0] = cache.indexA[1] then
   begin
      // Two points on B and one on A.
      m_type := e_separation_faceB;
      {$IFDEF D2009UP}
      localPointB1 := proxyB.m_vertices[cache.indexB[0]];
      localPointB2 := proxyB.m_vertices[cache.indexB[1]];
      {$ELSE}
      pB := proxyB.m_vertices;
      Inc(pB, cache.indexB[0]);
      localPointB1 := pB^;

      pB := proxyB.m_vertices;
      Inc(pB, cache.indexB[1]);
      localPointB2 := pB^;
      {$ENDIF}

      {$IFDEF OP_OVERLOAD}
      m_axis := b2Cross(localPointB2 - localPointB1, 1.0);
      m_axis.Normalize;
      {$ELSE}
      m_axis := b2Cross(Subtract(localPointB2, localPointB1), 1.0);
      Normalize(m_axis);
      {$ENDIF}

      normal := b2Mul(xfB.R, m_axis);
      m_localPoint := b2MiddlePoint(localPointB1, localPointB2);

      {$IFNDEF D2009UP}
      pA := proxyA.m_vertices;
      Inc(pA, cache.indexA[0]);
      {$ENDIF}

      {$IFDEF OP_OVERLOAD}
         {$IFDEF D2009UP}
         Result := b2Dot(b2Mul(xfA, proxyA.m_vertices[cache.indexA[0]]) -
            b2Mul(xfB, m_localPoint), normal);
         {$ELSE}
         Result := b2Dot(b2Mul(xfA, pA^) - b2Mul(xfB, m_localPoint), normal);
         {$ENDIF}
      {$ELSE}
         {$IFDEF D2009UP}
         Result := b2Dot(Subtract(b2Mul(xfA, proxyA.m_vertices[cache.indexA[0]]),
            b2Mul(xfB, m_localPoint)), normal);
         {$ELSE}
         Result := b2Dot(Subtract(b2Mul(xfA, pA^), b2Mul(xfB, m_localPoint)), normal);
         {$ENDIF}
      {$ENDIF}

      if Result < 0.0 then
      begin
         {$IFDEF OP_OVERLOAD}
         m_axis := -m_axis;
         {$ELSE}
         m_axis := Negative(m_axis);
         {$ENDIF}
         Result := -Result;
      end;
   end
   else
   begin
      // Two points on A and one or two points on B.
      m_type := e_separation_faceA;

      {$IFDEF D2009UP}
      localPointA1 := proxyA.m_vertices[cache.indexA[0]];
      localPointA2 := proxyA.m_vertices[cache.indexA[1]];
      {$ELSE}
      pA := proxyA.m_vertices;
      Inc(pA, cache.indexA[0]);
      localPointA1 := pA^;

      pA := proxyA.m_vertices;
      Inc(pA, cache.indexA[1]);
      localPointA2 := pA^;
      {$ENDIF}

      {$IFDEF OP_OVERLOAD}
      m_axis := b2Cross(localPointA2 - localPointA1, 1.0);
      m_axis.Normalize;
      {$ELSE}
      m_axis := b2Cross(Subtract(localPointA2, localPointA1), 1.0);
      Normalize(m_axis);
      {$ENDIF}

      normal := b2Mul(xfA.R, m_axis);
      m_localPoint := b2MiddlePoint(localPointA1, localPointA2);

      {$IFNDEF D2009UP}
      pB := proxyB.m_vertices;
      Inc(pB, cache.indexB[0]);
      {$ENDIF}

      {$IFDEF OP_OVERLOAD}
         {$IFDEF D2009UP}
         Result := b2Dot(b2Mul(xfB, proxyB.m_vertices[cache.indexB[0]]) -
            b2Mul(xfA, m_localPoint), normal);
         {$ELSE}
         Result := b2Dot(b2Mul(xfB, pB^) - b2Mul(xfA, m_localPoint), normal);
         {$ENDIF}
      {$ELSE}
         {$IFDEF D2009UP}
         Result := b2Dot(Subtract(b2Mul(xfB, proxyB.m_vertices[cache.indexB[0]]),
            b2Mul(xfA, m_localPoint)), normal);
         {$ELSE}
         Result := b2Dot(Subtract(b2Mul(xfB, pB^), b2Mul(xfA, m_localPoint)), normal);
         {$ENDIF}
      {$ENDIF}
      if Result < 0.0 then
      begin
         {$IFDEF OP_OVERLOAD}
         m_axis := -m_axis;
         {$ELSE}
         m_axis := Negative(m_axis);
         {$ENDIF}
         Result := -Result;
      end;
   end;
end;

function Tb2SeparationFunction.FindMinSeparation(var indexA, indexB: Int32; t: Float): Float;
var
   xfA, xfB: Tb2Transform;
   normal: TVector2;
   {$IFNDEF D2009UP}pA, pB: PVector2;{$ENDIF}
begin
   {$IFDEF OP_OVERLOAD}
   m_sweepA.GetTransform(xfA, t);
   m_sweepB.GetTransform(xfB, t);
   {$ELSE}
   GetTransform(m_sweepA, xfA, t);
   GetTransform(m_sweepB, xfB, t);
   {$ENDIF}

   case m_type of
      e_separation_points:
         begin
            {$IFDEF OP_OVERLOAD}
            indexA := m_proxyA^.GetSupport(b2MulT(xfA.R,  m_axis));
            indexB := m_proxyB^.GetSupport(b2MulT(xfB.R, -m_axis));
            {$ELSE}
            indexA := GetSupport(m_proxyA^, b2MulT(xfA.R,  m_axis));
            indexB := GetSupport(m_proxyB^, b2MulT(xfB.R, Negative(m_axis)));
            {$ENDIF}

            {$IFNDEF D2009UP}
            pA := m_proxyA^.m_vertices;
            pB := m_proxyB^.m_vertices;
            Inc(pA, indexA);
            Inc(pB, indexB);
            {$ENDIF}

            {$IFDEF OP_OVERLOAD}
               {$IFDEF D2009UP}
               Result := b2Dot(b2Mul(xfB, m_proxyB^.m_vertices[indexB]) -
                  b2Mul(xfA, m_proxyA^.m_vertices[indexA]), m_axis);
               {$ELSE}
               Result := b2Dot(b2Mul(xfB, pB^) - b2Mul(xfA, pA^), m_axis);
               {$ENDIF}
            {$ELSE}
               {$IFDEF D2009UP}
               Result:= b2Dot(Subtract(b2Mul(xfB, m_proxyB^.m_vertices[indexB]),
                  b2Mul(xfA, m_proxyA^.m_vertices[indexA])), m_axis);
               {$ELSE}
               Result:= b2Dot(Subtract(b2Mul(xfB, pB^), b2Mul(xfA, pA^)), m_axis);
               {$ENDIF}
            {$ENDIF}
         end;
      e_separation_faceA:
         begin
            normal := b2Mul(xfA.R, m_axis);
            indexA := -1;
            {$IFDEF OP_OVERLOAD}
            indexB := m_proxyB^.GetSupport(b2MulT(xfB.R, -normal));
            {$ELSE}
            indexB := GetSupport(m_proxyB^, b2MulT(xfB.R, Negative(normal)));
            {$ENDIF}

            {$IFNDEF D2009UP}
            pB := m_proxyB^.m_vertices;
            Inc(pB, indexB);
            {$ENDIF}

            {$IFDEF OP_OVERLOAD}
               {$IFDEF D2009UP}
               Result := b2Dot(b2Mul(xfB, m_proxyB^.m_vertices[indexB]) -
                  b2Mul(xfA, m_localPoint), normal);
               {$ELSE}
               Result := b2Dot(b2Mul(xfB, pB^) - b2Mul(xfA, m_localPoint), normal);
               {$ENDIF}
            {$ELSE}
               {$IFDEF D2009UP}
               Result := b2Dot(Subtract(b2Mul(xfB, m_proxyB^.m_vertices[indexB]),
                  b2Mul(xfA, m_localPoint)), normal);
               {$ELSE}
               Result := b2Dot(Subtract(b2Mul(xfB, pB^), b2Mul(xfA, m_localPoint)), normal);
               {$ENDIF}
            {$ENDIF}
         end;
      e_separation_faceB:
         begin
            normal := b2Mul(xfB.R, m_axis);

            indexB := -1;
            {$IFDEF OP_OVERLOAD}
            indexA := m_proxyA.GetSupport(b2MulT(xfA.R, -normal));
            {$ELSE}
            indexA := GetSupport(m_proxyA^, b2MulT(xfA.R, Negative(normal)));
            {$ENDIF}

            {$IFNDEF D2009UP}
            pA := m_proxyA^.m_vertices;
            Inc(pA, indexA);
            {$ENDIF}

            {$IFDEF OP_OVERLOAD}
               {$IFDEF D2009UP}
               Result := b2Dot(b2Mul(xfA, m_proxyA.m_vertices[indexA]) -
                  b2Mul(xfB, m_localPoint), normal);
               {$ELSE}
               Result := b2Dot(b2Mul(xfA, pA^) - b2Mul(xfB, m_localPoint), normal);
               {$ENDIF}
            {$ELSE}
               {$IFDEF D2009UP}
               Result := b2Dot(Subtract(b2Mul(xfA, m_proxyA.m_vertices[indexA]),
                  b2Mul(xfB, m_localPoint)), normal);
               {$ELSE}
               Result := b2Dot(Subtract(b2Mul(xfA, pA^), b2Mul(xfB, m_localPoint)), normal);
               {$ENDIF}
            {$ENDIF}
         end;
   else
      //b2Assert(false);
      indexA := -1;
      indexB := -1;
      Result := 0.0;
   end;
end;

function Tb2SeparationFunction.Evaluate(indexA, indexB: Int32; t: Float): Float;
var
   xfA, xfB: Tb2Transform;
   {$IFNDEF D2009UP}pA, pB: PVector2;{$ENDIF}
begin
   {$IFDEF OP_OVERLOAD}
   m_sweepA.GetTransform(xfA, t);
   m_sweepB.GetTransform(xfB, t);
   {$ELSE}
   GetTransform(m_sweepA, xfA, t);
   GetTransform(m_sweepB, xfB, t);
   {$ENDIF}

   case m_type of
      e_separation_points:
         begin
            {$IFNDEF D2009UP}
            pA := m_proxyA^.m_vertices;
            pB := m_proxyB^.m_vertices;
            Inc(pA, indexA);
            Inc(pB, indexB);
            {$ENDIF}

            {$IFDEF OP_OVERLOAD}
               {$IFDEF D2009UP}
               Result := b2Dot(b2Mul(xfB, m_proxyB^.m_vertices[indexB]) -
                  b2Mul(xfA, m_proxyA^.m_vertices[indexA]), m_axis);
               {$ELSE}
               Result := b2Dot(b2Mul(xfB, pB^) - b2Mul(xfA, pA^), m_axis);
               {$ENDIF}
            {$ELSE}
               {$IFDEF D2009UP}
               Result := b2Dot(Subtract(b2Mul(xfB, m_proxyB^.m_vertices[indexB]),
                  b2Mul(xfA, m_proxyA.m_vertices[indexA])), m_axis);
               {$ELSE}
               Result := b2Dot(Subtract(b2Mul(xfB, pB^), b2Mul(xfA, pA^)), m_axis);
               {$ENDIF}
            {$ENDIF}
         end;
      e_separation_faceA:
         begin
            {$IFNDEF D2009UP}
            pB := m_proxyB^.m_vertices;
            Inc(pB, indexB);
            {$ENDIF}

            {$IFDEF OP_OVERLOAD}
               {$IFDEF D2009UP}
               Result := b2Dot(b2Mul(xfB, m_proxyB^.m_vertices[indexB]) -
                  b2Mul(xfA, m_localPoint), b2Mul(xfA.R, m_axis));
               {$ELSE}
               Result := b2Dot(b2Mul(xfB, pB^) - b2Mul(xfA, m_localPoint), b2Mul(xfA.R, m_axis));
               {$ENDIF}
            {$ELSE}
               {$IFDEF D2009UP}
               Result := b2Dot(Subtract(b2Mul(xfB, m_proxyB^.m_vertices[indexB]),
                  b2Mul(xfA, m_localPoint)), b2Mul(xfA.R, m_axis));
               {$ELSE}
               Result := b2Dot(Subtract(b2Mul(xfB, pB^),
                  b2Mul(xfA, m_localPoint)), b2Mul(xfA.R, m_axis));
               {$ENDIF}
            {$ENDIF}
         end;
      e_separation_faceB:
         begin
            {$IFNDEF D2009UP}
            pA := m_proxyA^.m_vertices;
            Inc(pA, indexA);
            {$ENDIF}

            {$IFDEF OP_OVERLOAD}
               {$IFDEF D2009UP}
               Result := b2Dot(b2Mul(xfA, m_proxyA^.m_vertices[indexA]) -
                  b2Mul(xfB, m_localPoint), b2Mul(xfB.R, m_axis));
               {$ELSE}
               Result := b2Dot(b2Mul(xfA, pA^) - b2Mul(xfB, m_localPoint), b2Mul(xfB.R, m_axis));
               {$ENDIF}
            {$ELSE}
               {$IFDEF D2009UP}
               Result := b2Dot(Subtract(b2Mul(xfA, m_proxyA^.m_vertices[indexA]),
                  b2Mul(xfB, m_localPoint)), b2Mul(xfB.R, m_axis));
               {$ELSE}
               Result := b2Dot(Subtract(b2Mul(xfA, pA^),
                  b2Mul(xfB, m_localPoint)), b2Mul(xfB.R, m_axis));
               {$ENDIF}
            {$ENDIF}
         end;
   else
      //b2Assert(false);
      Result := 0.0;
   end;
end;

var
   toi_separation_fcn: Tb2SeparationFunction;
/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collision. If you change the time interval, you should call this function again.
/// Note: use b2Distance to compute the contact point and normal at the time of impact.
function b2TimeOfImpact(var output: Tb2TOIOutput; const input: Tb2TOIInput): Float;
const
   c_tolerance = 0.25 * b2_linearSlop;
   k_maxIterations = 20;	// TODO_ERIN b2Settings
var
   sweepA, sweepB: Tb2Sweep;
   t1, s1, t2, s2, a1, a2, t, s, totalRadius, target, tolerance: Float;
   iter, pushBackIter: Int32;
   cache: Tb2SimplexCache;
   distanceInput: Tb2DistanceInput;
   distanceOutput: Tb2DistanceOutput;
   xfA, xfB: Tb2Transform;
   done: Boolean;
   indexA, indexB, rootIterCount: Int32;
begin
   Inc(b2_toiCalls);

   output.state := e_toi_unknown;
   output.t := input.tMax;

   sweepA := input.sweepA;
   sweepB := input.sweepB;

   // Large rotations can make the root finder fail, so we normalize the sweep angles.
   {$IFDEF OP_OVERLOAD}
   sweepA.Normalize;
   sweepB.Normalize;
   {$ELSE}
   Normalize(sweepA);
   Normalize(sweepB);
   {$ENDIF}

   totalRadius := input.proxyA.m_radius + input.proxyB.m_radius;
   target := b2Max(b2_linearSlop, totalRadius - 3.0 * b2_linearSlop);
   tolerance := 0.25 * b2_linearSlop;
   //b2Assert(target > tolerance);

   t1 := 0.0;
   iter := 0;

   // Prepare input for distance query.
   cache.count := 0;
   distanceInput.proxyA := input.proxyA;
   distanceInput.proxyB := input.proxyB;
   distanceInput.useRadii := False;

   // The outer loop progressively attempts to compute new separating axes.
   // This loop terminates when an axis is repeated (no progress is made).
   while True do
   begin
      {$IFDEF OP_OVERLOAD}
      sweepA.GetTransform(xfA, t1);
      sweepB.GetTransform(xfB, t1);
      {$ELSE}
      GetTransform(sweepA, xfA, t1);
      GetTransform(sweepB, xfB, t1);
      {$ENDIF}

      // Get the distance between shapes. We can also use the results
      // to get a separating axis.
      distanceInput.transformA := xfA;
      distanceInput.transformB := xfB;
      b2Distance(distanceOutput, cache, distanceInput);

      // If the shapes are overlapped, we give up on continuous collision.
      if distanceOutput.distance <= 0.0 then
      begin
         // Failure!
         output.state := e_toi_overlapped;
         output.t := 0.0;
         Break;
      end;

      if distanceOutput.distance < target + tolerance then
      begin
         // Victory!
         output.state := e_toi_touching;
         output.t := t1;
         Break;
      end;

      // Initialize the separating axis.
      toi_separation_fcn.Initialize(cache, input.proxyA, input.proxyB, sweepA, sweepB, t1);

      // Compute the TOI on the separating axis. We do this by successively
      // resolving the deepest point. This loop is bounded by the number of vertices.
      done := False;
      t2 := input.tMax;
      pushBackIter := 0;
      while True do
      begin
         // Find the deepest point at t2. Store the witness point indices.
         s2 := toi_separation_fcn.FindMinSeparation(indexA, indexB, t2);

         // Is the final configuration separated?
         if s2 > target + tolerance then
         begin
            // Victory!
            output.state := e_toi_separated;
            output.t := input.tMax;
            done := True;
            Break;
         end;

         // Has the separation reached tolerance?
         if s2 > target - tolerance then
         begin
            // Advance the sweeps
            t1 := t2;
            Break;
         end;

         // Compute the initial separation of the witness points.
         s1 := toi_separation_fcn.Evaluate(indexA, indexB, t1);

         // Check for initial overlap. This might happen if the root finder
         // runs out of iterations.
         if s1 < target - tolerance then
         begin
            output.state := e_toi_failed;
            output.t := t1;
            done := True;
            Break;
         end;

         // Check for touching
         if s1 <= target + tolerance then
         begin
            // Victory! t1 should hold the TOI (could be 0.0).
            output.state := e_toi_touching;
            output.t := t1;
            done := True;
            Break;
         end;

         // Compute 1D root of: f(x) - target := 0
         rootIterCount := 0;
         a1 := t1;
         a2 := t2;
         while True do
         begin
            // Use a mix of the secant rule and bisection.
            if rootIterCount and 1 <> 0 then
            begin
               // Secant rule to improve convergence.
               t := a1 + (target - s1) * (a2 - a1) / (s2 - s1);
            end
            else
            begin
               // Bisection to guarantee progress.
               t := 0.5 * (a1 + a2);
            end;

            s := toi_separation_fcn.Evaluate(indexA, indexB, t);

            if Abs(s - target) < tolerance then
            begin
               // t2 holds a tentative value for t1
               t2 := t;
               Break;
            end;

            // Ensure we continue to bracket the root.
            if s > target then
            begin
               a1 := t;
               s1 := s;
            end
            else
            begin
               a2 := t;
               s2 := s;
            end;

            Inc(rootIterCount);
            Inc(b2_toiRootIters);

            if rootIterCount = 50 then
               Break;
         end;

         b2_toiMaxRootIters := b2Max(b2_toiMaxRootIters, rootIterCount);
         Inc(pushBackIter);

         if pushBackIter = b2_maxPolygonVertices then
            Break;
      end;

      Inc(iter);
      Inc(b2_toiIters);

      if done then
         Break;

      if iter = k_maxIterations then
      begin
         // Root finder got stuck. Semi-victory.
         output.state := e_toi_failed;
         output.t := t1;
         Break;
      end;
   end;

   b2_toiMaxIters := b2Max(b2_toiMaxIters, iter);
end;

{ b2Distance.cpp }

type
   Pb2SimplexVertex = ^Tb2SimplexVertex;
   Tb2SimplexVertex = record
      wA: TVector2;		// support point in proxyA
      wB: TVector2;		// support point in proxyB
      w: TVector2;		// wB - wA
      a: Float;		// barycentric coordinate for closest point
      indexA: Int32;	// wA index
      indexB: Int32;	// wB index
   end;

   Tb2Simplex = class
   public
      m_v1, m_v2, m_v3: Tb2SimplexVertex;
      m_count: Int32;

	    procedure ReadCache(const cache: Tb2SimplexCache; const proxyA,
         proxyB: Tb2DistanceProxy; const transformA, transformB: Tb2Transform);
      procedure WriteCache(var cache: Tb2SimplexCache);

      function GetSearchDirection: TVector2;
      function GetClosestPoint: TVector2;
      procedure GetWitnessPoints(var pA, pB: TVector2);
      function GetMetric: Float;

      procedure Solve2;
	    procedure Solve3;
   end;

{ Tb2Simplex }

procedure Tb2Simplex.ReadCache(const cache: Tb2SimplexCache; const proxyA,
   proxyB: Tb2DistanceProxy; const transformA, transformB: Tb2Transform);
var
   vertices, v: Pb2SimplexVertex;
   i: Integer;
   metric1, metric2: Float;
   {$IFNDEF D2009UP}pA, pB: PVector2;{$ENDIF}
begin
   //b2Assert(cache.count <= 3);

   // Copy data from cache.
   m_count := cache.count;
   vertices := @m_v1;
   for i := 0 to m_count - 1 do
   begin
      v := vertices;
      Inc(v, i);

      with v^ do
      begin
         indexA := cache.indexA[i];
         indexB := cache.indexB[i];

         {$IFDEF D2009UP}
         wA := b2Mul(transformA, proxyA.m_vertices[indexA]);
         wB := b2Mul(transformB, proxyB.m_vertices[indexB]);
         {$ELSE}
         pA := proxyA.m_vertices;
         pB := proxyB.m_vertices;
         Inc(pA, indexA);
         Inc(pB, indexB);
         wA := b2Mul(transformA, pA^);
         wB := b2Mul(transformB, pB^);
         {$ENDIF}

         {$IFDEF OP_OVERLOAD}
         w := wB - wA;
         {$ELSE}
         w := Subtract(wB, wA);
         {$ENDIF}
         a := 0.0;
      end;
   end;

   // Compute the new simplex metric, if it is substantially different than
   // old metric then flush the simplex.
   if m_count > 1 then
   begin
      metric1 := cache.metric;
      metric2 := GetMetric;
      if (metric2 < 0.5 * metric1) or (2.0 * metric1 < metric2) or (metric2 < FLT_epsilon) then
      begin
         // Reset the simplex.
         m_count := 0;
      end;
   end;

   // If the cache is empty or invalid ...
   if m_count = 0 then
      with m_v1 do
      begin
         indexA := 0;
         indexB := 0;
         {$IFDEF D2009UP}
         wA := b2Mul(transformA, proxyA.m_vertices[0]);
         wB := b2Mul(transformB, proxyB.m_vertices[0]);
         {$ELSE}
         wA := b2Mul(transformA, proxyA.m_vertices^);
         wB := b2Mul(transformB, proxyB.m_vertices^);
         {$ENDIF}

         {$IFDEF OP_OVERLOAD}
         w := wB - wA;
         {$ELSE}
         w := Subtract(wB, wA);
         {$ENDIF}
         m_count := 1;
      end;
end;

procedure Tb2Simplex.WriteCache(var cache: Tb2SimplexCache);
var
   i: Integer;
   vertices, v: Pb2SimplexVertex;
begin
   cache.metric := GetMetric;
   cache.count := m_count;
   vertices := @m_v1;
   for i := 0 to m_count - 1 do
   begin
      v := vertices;
      Inc(v, i);
      cache.indexA[i] := v^.indexA;
      cache.indexB[i] := v^.indexB;
   end;
end;

function Tb2Simplex.GetSearchDirection: TVector2;
var
   e12: TVector2;
begin
   case m_count of
      1:
         {$IFDEF OP_OVERLOAD}
         Result := -m_v1.w;
         {$ELSE}
         Result := Negative(m_v1.w);
         {$ENDIF}
      2:
         begin
            {$IFDEF OP_OVERLOAD}
            e12 := m_v2.w - m_v1.w;
            {$ELSE}
            e12 := Subtract(m_v2.w, m_v1.w);
            {$ENDIF}
            if b2Cross(e12, {$IFDEF OP_OVERLOAD}-m_v1.w{$ELSE}Negative(m_v1.w){$ENDIF}) > 0.0 then
            begin
               // Origin is left of e12.
               Result := b2Cross(1.0, e12);
            end
            else
            begin
              // Origin is right of e12.
              Result := b2Cross(e12, 1.0);
            end;
         end;
   else
      //b2Assert(false);
      Result := b2Vec2_zero;
   end;
end;

function Tb2Simplex.GetClosestPoint: TVector2;
begin
   case m_count of
      1: Result := m_v1.w;
      2: Result :=
            {$IFDEF OP_OVERLOAD}
            m_v1.a * m_v1.w + m_v2.a * m_v2.w;
            {$ELSE}
            Add(Multiply(m_v1.w, m_v1.a), Multiply(m_v2.w, m_v2.a));
            {$ENDIF}
      3: Result := b2Vec2_zero;
   else
      //b2Assert(false);
      Result := b2Vec2_zero;
   end;
end;

procedure Tb2Simplex.GetWitnessPoints(var pA, pB: TVector2);
begin
   case m_count of
      1:
         begin
            pA := m_v1.wA;
            pB := m_v1.wB;
         end;
      2:
         begin
            {$IFDEF OP_OVERLOAD}
            pA := m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
            pB := m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
            {$ELSE}
            pA := Add(Multiply(m_v1.wA, m_v1.a), Multiply(m_v2.wA, m_v2.a));
            pB := Add(Multiply(m_v1.wB, m_v1.a), Multiply(m_v2.wB, m_v2.a));
            {$ENDIF}
         end;
      3:
         begin
            {$IFDEF OP_OVERLOAD}
            pA := m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
            {$ELSE}
            pA := Add(Multiply(m_v1.wA, m_v1.a), Multiply(m_v2.wA, m_v2.a),
               Multiply(m_v3.wA, m_v3.a));
            {$ENDIF}
            pB := pA;
         end;
   else
      //b2Assert(false);
   end;
end;

function Tb2Simplex.GetMetric: Float;
begin
   case m_count of
      0:
         begin
            //b2Assert(false);
            Result := 0.0;
         end;
      1: Result := 0.0;
      2: Result := UPhysics2DTypes.b2Distance(m_v1.w, m_v2.w);
      3: Result :=
         {$IFDEF OP_OVERLOAD}
         b2Cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w)
         {$ELSE}
         b2Cross(Subtract(m_v2.w, m_v1.w), Subtract(m_v3.w, m_v1.w))
         {$ENDIF}
   else
      //b2Assert(false);
      Result := 0.0;
   end;
end;

// Solve a line segment using barycentric coordinates.
//
// p := a1 * w1 + a2 * w2
// a1 + a2 := 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 := w2 - w1
// dot(p, e) := 0
// a1 * dot(w1, e) + a2 * dot(w2, e) := 0
//
// 2-by-2 linear system
// [1      1     ][a1] := [1]
// [w1.e12 w2.e12][a2] := [0]
//
// Define
// d12_1 :=  dot(w2, e12)
// d12_2 := -dot(w1, e12)
// d12 := d12_1 + d12_2
//
// Solution
// a1 := d12_1 / d12
// a2 := d12_2 / d12
procedure Tb2Simplex.Solve2;
var
   e12: TVector2;
   d12_2, d12_1, inv_d12: Float;
begin
   {$IFDEF OP_OVERLOAD}
   e12 := m_v2.w - m_v1.w;
   {$ELSE}
   e12 := Subtract(m_v2.w, m_v1.w);
   {$ENDIF}

   // w1 region
   d12_2 := -b2Dot(m_v1.w, e12);
   if d12_2 <= 0.0 then
   begin
      // a2 <= 0, so we clamp it to 0
      m_v1.a := 1.0;
      m_count := 1;
      Exit;
   end;

   // w2 region
   d12_1 := b2Dot(m_v2.w, e12);
   if d12_1 <= 0.0 then
   begin
      // a1 <= 0, so we clamp it to 0
      m_v2.a := 1.0;
      m_count := 1;
      m_v1 := m_v2;
      Exit;
   end;

   // Must be in e12 region.
   inv_d12 := 1.0 / (d12_1 + d12_2);
   m_v1.a := d12_1 * inv_d12;
   m_v2.a := d12_2 * inv_d12;
   m_count := 2;
end;

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
procedure Tb2Simplex.Solve3;
var
   w1, w2, w3: TVector2;
   e12, e13, e23: TVector2;
   d12_1, d12_2, d13_1, d13_2, d23_1, d23_2,
   n123, d123_1, d123_2, d123_3,
   inv_d: Float;
begin
   w1 := m_v1.w;
   w2 := m_v2.w;
   w3 := m_v3.w;

   // Edge12
   // [1      1     ][a1] := [1]
   // [w1.e12 w2.e12][a2] := [0]
   // a3 := 0
   {$IFDEF OP_OVERLOAD}
   e12 := w2 - w1;
   {$ELSE}
   e12 := Subtract(w2, w1);
   {$ENDIF}
   d12_1 := b2Dot(w2, e12);
   d12_2 := -b2Dot(w1, e12);

   // Edge13
   // [1      1     ][a1] := [1]
   // [w1.e13 w3.e13][a3] := [0]
   // a2 := 0
   {$IFDEF OP_OVERLOAD}
   e13 := w3 - w1;
   {$ELSE}
   e13 := Subtract(w3, w1);
   {$ENDIF}
   d13_1 := b2Dot(w3, e13);
   d13_2 := -b2Dot(w1, e13);

   // Edge23
   // [1      1     ][a2] := [1]
   // [w2.e23 w3.e23][a3] := [0]
   // a1 := 0
   {$IFDEF OP_OVERLOAD}
   e23 := w3 - w2;
   {$ELSE}
   e23 := Subtract(w3, w2);
   {$ENDIF}
   d23_1 := b2Dot(w3, e23);
   d23_2 := -b2Dot(w2, e23);

   // Triangle123
   n123 := b2Cross(e12, e13);

   d123_1 := n123 * b2Cross(w2, w3);
   d123_2 := n123 * b2Cross(w3, w1);
   d123_3 := n123 * b2Cross(w1, w2);

   // w1 region
   if (d12_2 <= 0.0) and (d13_2 <= 0.0) then
   begin
      m_v1.a := 1.0;
      m_count := 1;
      Exit;
   end;

   // e12
   if (d12_1 > 0.0) and (d12_2 > 0.0) and (d123_3 <= 0.0) then
   begin
      inv_d := 1.0 / (d12_1 + d12_2);
      m_v1.a := d12_1 * inv_d;
      m_v2.a := d12_2 * inv_d;
      m_count := 2;
      Exit;
   end;

   // e13
   if (d13_1 > 0.0) and (d13_2 > 0.0) and (d123_2 <= 0.0) then
   begin
      inv_d := 1.0 / (d13_1 + d13_2);
      m_v1.a := d13_1 * inv_d;
      m_v3.a := d13_2 * inv_d;
      m_count := 2;
      m_v2 := m_v3;
      Exit;
   end;

   // w2 region
   if (d12_1 <= 0.0) and (d23_2 <= 0.0) then
   begin
      m_v2.a := 1.0;
      m_count := 1;
      m_v1 := m_v2;
      Exit;
   end;

   // w3 region
   if (d13_1 <= 0.0) and (d23_1 <= 0.0) then
   begin
      m_v3.a := 1.0;
      m_count := 1;
      m_v1 := m_v3;
      Exit;
   end;

   // e23
   if (d23_1 > 0.0) and (d23_2 > 0.0) and (d123_1 <= 0.0) then
   begin
      inv_d := 1.0 / (d23_1 + d23_2);
      m_v2.a := d23_1 * inv_d;
      m_v3.a := d23_2 * inv_d;
      m_count := 2;
      m_v1 := m_v3;
      Exit;
   end;

   // Must be in triangle123
   inv_d := 1.0 / (d123_1 + d123_2 + d123_3);
   m_v1.a := d123_1 * inv_d;
   m_v2.a := d123_2 * inv_d;
   m_v3.a := d123_3 * inv_d;
   m_count := 3;
end;

var
   distance_simplex: Tb2Simplex;
procedure b2Distance(var output: Tb2DistanceOutput;	var cache: Tb2SimplexCache;
   const input: Tb2DistanceInput);
const
   k_maxIters = 20;
var
   i: Integer;
   saveA, saveB: array[0..2] of Int32;
   saveCount, iter: Int32;
   vertices, vertex: Pb2SimplexVertex;
   distanceSqr1, distanceSqr2, rA, rB: Float;
   d, normal: TVector2;
   duplicate: Boolean;
   {$IFNDEF D2009UP}pA, pB: PVector2;{$ENDIF}
begin
   Inc(b2_gjkCalls);

   // Initialize the simplex.
   distance_simplex.ReadCache(cache, input.proxyA, input.proxyB,
      input.transformA, input.transformB);

   // Get simplex vertices as an array.
   vertices := @distance_simplex.m_v1;

   // These store the vertices of the last simplex so that we
   // can check for duplicates and prevent cycling.
   //saveCount := 0;

   {$IFDEF OP_OVERLOAD}
   distanceSqr1 := distance_simplex.GetClosestPoint.SqrLength;
   {$ELSE}
   distanceSqr1 := SqrLength(distance_simplex.GetClosestPoint);
   {$ENDIF}
   //distanceSqr2 := distanceSqr1;

   // Main iteration loop.
   iter := 0;
   while iter < k_maxIters do
   begin
      // Copy simplex so we can identify duplicates.
      saveCount := distance_simplex.m_count;
      for i := 0 to saveCount - 1 do
      begin
         vertex := vertices;
         Inc(vertex, i);
         with vertex^ do
         begin
            saveA[i] := indexA;
            saveB[i] := indexB;
         end;
      end;

      case distance_simplex.m_count of
         2: distance_simplex.Solve2;
         3: distance_simplex.Solve3;
      else
         //b2Assert(false);
      end;

      // If we have 3 points, then the origin is in the corresponding triangle.
      if distance_simplex.m_count = 3 then
         Break;

      // Compute closest point.
      {$IFDEF OP_OVERLOAD}
      distanceSqr2 := distance_simplex.GetClosestPoint.SqrLength;
      {$ELSE}
      distanceSqr2 := SqrLength(distance_simplex.GetClosestPoint);
      {$ENDIF}

      // Ensure progress
      if distanceSqr2 >= distanceSqr1 then
      begin
        //break;
      end;
      distanceSqr1 := distanceSqr2;

      // Get search direction.
      d := distance_simplex.GetSearchDirection;

      // Ensure the search direction is numerically fit.
      if {$IFDEF OP_OVERLOAD}d.SqrLength{$ELSE}SqrLength(d){$ENDIF} <
         FLT_EPSILON * FLT_EPSILON then
      begin
         // The origin is probably contained by a line segment
         // or triangle. Thus the shapes are overlapped.

         // We can't return zero here even though there may be overlap.
         // In case the simplex is a point, segment, or triangle it is difficult
         // to determine if the origin is contained in the CSO or very close to it.
         Break;
      end;

      // Compute a tentative new simplex vertex using support points.
      vertex := vertices;
      Inc(vertex, distance_simplex.m_count);
      with vertex^ do
      begin
         {$IFDEF OP_OVERLOAD}
         indexA := input.proxyA.GetSupport(b2MulT(input.transformA.R, -d));
         indexB := input.proxyB.GetSupport(b2MulT(input.transformB.R, d));
         {$ELSE}
         indexA := GetSupport(input.proxyA, b2MulT(input.transformA.R, Negative(d)));
         indexB := GetSupport(input.proxyB, b2MulT(input.transformB.R, d));
         {$ENDIF}

         {$IFDEF D2009UP}
         wA := b2Mul(input.transformA, input.proxyA.m_vertices[indexA]);
         wB := b2Mul(input.transformB, input.proxyB.m_vertices[indexB]);
         {$ELSE}
         pA := input.proxyA.m_vertices;
         pB := input.proxyB.m_vertices;
         Inc(pA, indexA);
         Inc(pB, indexB);
         wA := b2Mul(input.transformA, pA^);
         wB := b2Mul(input.transformB, pB^);
         {$ENDIF}

         {$IFDEF OP_OVERLOAD}
         w := wB - wA;
         {$ELSE}
         w := Subtract(wB, wA);
         {$ENDIF}
      end;

      // Iteration count is equated to the number of support point calls.
      Inc(iter);
      Inc(b2_gjkIters);

      // Check for duplicate support points. This is the main termination criteria.
      duplicate := False;
      for i := 0 to saveCount - 1 do
      begin
         if (vertex.indexA = saveA[i]) and (vertex.indexB = saveB[i]) then
         begin
            duplicate := True;
            Break;
         end;
      end;

      // If we found a duplicate support point we must exit to avoid cycling.
      if duplicate then
         Break;

      // New vertex is ok and needed.
      Inc(distance_simplex.m_count);
   end;

   b2_gjkMaxIters := b2Max(b2_gjkMaxIters, iter);

   // Prepare output.
   with output do
   begin
      distance_simplex.GetWitnessPoints(pointA, pointB);
      distance := UPhysics2DTypes.b2Distance(pointA, pointB);
      iterations := iter;
   end;

   // Cache the simplex.
   distance_simplex.WriteCache(cache);

   // Apply radii if requested.
   if input.useRadii then
   begin
      rA := input.proxyA.m_radius;
      rB := input.proxyB.m_radius;

      with output do
      begin
         if (distance > rA + rB) and (distance > FLT_EPSILON) then
         begin
            // Shapes are still no overlapped.
            // Move the witness points to the outer surface.
            distance := distance - (rA + rB);
            {$IFDEF OP_OVERLOAD}
            normal := pointB - pointA;
            normal.Normalize;
            pointA.AddBy(rA * normal);
            pointB.SubtractBy(rB * normal);
            {$ELSE}
            normal := Subtract(pointB, pointA);
            Normalize(normal);
            AddBy(pointA, Multiply(normal, rA));
            SubtractBy(pointB, Multiply(normal, rB));
            {$ENDIF}
         end
         else
         begin
            // Shapes are overlapped when radii are considered.
            // Move the witness points to the middle.
            pointA := b2MiddlePoint(pointA, pointB);
            pointB := pointA;
            distance := 0.0;
         end;
      end;
   end;
end;

{ b2Collision.cpp }
function b2TestOverlap(const a, b: Tb2AABB): Boolean; overload;
var
   d1, d2: TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   d1 := b.lowerBound - a.upperBound;
   d2 := a.lowerBound - b.upperBound;
   {$ELSE}
   d1 := Subtract(b.lowerBound, a.upperBound);
   d2 := Subtract(a.lowerBound, b.upperBound);
   {$ENDIF}

   if (d1.x > 0.0) or (d1.y > 0.0) or (d2.x > 0.0) or (d2.y > 0.0) then
      Result := False
   else
      Result := True;
end;

/// Determine if two generic shapes overlap.
function b2TestOverlap(shapeA, shapeB: Tb2Shape; indexA, indexB: Int32;
   const xfA, xfB: Tb2Transform): Boolean; overload;
var
   input: Tb2DistanceInput;
   output: Tb2DistanceOutput;
   cache: Tb2SimplexCache;
begin
   with input do
   begin
      {$IFDEF OP_OVERLOAD}
      proxyA.SetShape(shapeA, indexA);
      proxyB.SetShape(shapeB, indexB);
      {$ELSE}
      SetShape(proxyA, shapeA, indexA);
      SetShape(proxyB, shapeB, indexB);
      {$ENDIF}
      transformA := xfA;
      transformB := xfB;
      useRadii := True;
   end;

   cache.count := 0;
   b2Distance(output, cache, input);
   Result := output.distance < 10.0 * FLT_EPSILON;
end;

/////////////////////////////////////////////////////////////////////////////
{$IFDEF OP_OVERLOAD}
procedure Tb2WorldManifold.Initialize(const manifold: Tb2Manifold;
   const xfA, xfB: Tb2Transform; radiusA, radiusB: Float);
var
   i: Integer;
   pointA, pointB, cA, cB, planePoint, clipPoint: TVector2;
begin
   with manifold do
   begin
      if pointCount = 0 then
         Exit;

      case manifoldType of
         e_manifold_circles:
            begin
               normal.x := 1.0;
               normal.y := 0.0;

               pointA := b2Mul(xfA, localPoint);
               pointB := b2Mul(xfB, points[0].localPoint{manifold.points});
               if b2DistanceSquared(pointA, pointB) > FLT_EPSILON * FLT_EPSILON then
               begin
                  normal := pointB - pointA;
                  normal.Normalize;
               end;

               cA := pointA + radiusA * normal;
               cB := pointB - radiusB * normal;
               Self.points[0] := 0.5 * (cA + cB);
            end;
         e_manifold_faceA:
            begin
               normal := b2Mul(xfA.R, localNormal);
               planePoint := b2Mul(xfA, localPoint);

               for i := 0 to pointCount - 1 do
               begin
                  clipPoint := b2Mul(xfB, points[i].localPoint{manifold.points});
                  cA := clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
                  cB := clipPoint - radiusB * normal;
                  Self.points[i] := 0.5 * (cA + cB);
               end;
            end;
         e_manifold_faceB:
            begin
               normal := b2Mul(xfB.R, localNormal);
               planePoint := b2Mul(xfB, localPoint);

               for i := 0 to pointCount - 1 do
               begin
                  clipPoint := b2Mul(xfA, points[i].localPoint{manifold.points});
                  cB := clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
                  cA := clipPoint - radiusA * normal;
                  Self.points[i] := 0.5 * (cA + cB);
               end;

               // Ensure normal points from A to B.
               normal := -normal;
            end;
      end;
   end;
end;
{$ENDIF}

{ Tb2AABB }
{$IFDEF OP_OVERLOAD}
function Tb2AABB.IsValid: Boolean;
var
   d: TVector2;
begin
   d := upperBound - lowerBound;
   Result := (d.x >= 0.0) and (d.y >= 0.0) and upperBound.IsValid and lowerBound.IsValid;
end;

function Tb2AABB.GetCenter: TVector2;
begin
   Result := b2MiddlePoint(lowerBound, upperBound);
end;

function Tb2AABB.GetExtents: TVector2;
begin
   Result := (upperBound - lowerBound) * 0.5;
end;

function Tb2AABB.GetPerimeter: Float;
begin
   Result := 2.0 * ((upperBound.x - lowerBound.x) + (upperBound.y - lowerBound.y));
end;

procedure Tb2AABB.Combine(const aabb: Tb2AABB);
begin
	 lowerBound := b2Min(lowerBound, aabb.lowerBound);
	 upperBound := b2Max(upperBound, aabb.upperBound);
end;

procedure Tb2AABB.Combine(const aabb1, aabb2: Tb2AABB);
begin
   lowerBound := b2Min(aabb1.lowerBound, aabb2.lowerBound);
   upperBound := b2Max(aabb1.upperBound, aabb2.upperBound);
end;

function Tb2AABB.Contains(const aabb: Tb2AABB): Boolean;
begin
   Result := True;
   Result := Result and (lowerBound.x <= aabb.lowerBound.x);
   Result := Result and (lowerBound.y <= aabb.lowerBound.y);
   Result := Result and (aabb.upperBound.x <= upperBound.x);
   Result := Result and (aabb.upperBound.y <= upperBound.y);
end;

function Tb2AABB.RayCast(var output: Tb2RayCastOutput;
   const input: Tb2RayCastInput): Boolean;
var
   i: Integer;
   tmin, tmax, inv_d, t1, t2, s: Float;
   p, d, absD, normal: TVector2;
begin
   tmin := -FLT_MAX;
   tmax := FLT_MAX;

   p := input.p1;
   d := input.p2 - input.p1;
   absD := b2Abs(d);

   for i := 0 to 1 do
   begin
      if TVector2Arraied(absD)[i] < FLT_EPSILON then
      begin
         // Parallel.
         if (TVector2Arraied(p)[i] < TVector2Arraied(lowerBound)[i]) or
            (TVector2Arraied(upperBound)[i] < TVector2Arraied(p)[i]) then
         begin
            Result := False;
            Exit;
         end;
      end
      else
      begin
         inv_d := 1.0 / TVector2Arraied(d)[i];
         t1 := (TVector2Arraied(lowerBound)[i] - TVector2Arraied(p)[i]) * inv_d;
         t2 := (TVector2Arraied(upperBound)[i] - TVector2Arraied(p)[i]) * inv_d;

         // Sign of the normal vector.
         s := -1.0;

         if t1 > t2 then
         begin
            b2Swap(t1, t2);
            s := 1.0;
         end;

         // Push the min up
         if t1 > tmin then
         begin
            normal := b2Vec2_Zero;
            TVector2Arraied(normal)[i] := s;
            tmin := t1;
         end;

         // Pull the max down
         tmax := b2Min(tmax, t2);

         if tmin > tmax then
         begin
            Result := False;
            Exit;
         end;
      end;
   end;

   // Does the ray start inside the box?
   // Does the ray intersect beyond the max fraction?
   if (tmin < 0.0) or (input.maxFraction < tmin) then
   begin
      Result := False;
      Exit;
   end;

   // Intersection.
   output.fraction := tmin;
   output.normal := normal;
   Result := True;
end;

{$ENDIF}

{ Tb2DebugDraw }

constructor Tb2DebugDraw.Create;
begin
   m_drawFlags := [];
   m_shapeColor_Inactive := MakeColor(0.5, 0.5, 0.3);
   m_shapeColor_Static := MakeColor(0.5, 0.9, 0.5);
   m_shapeColor_Kinematic := MakeColor(0.5, 0.5, 0.9);
   m_shapeColor_Sleeping := MakeColor(0.6, 0.6, 0.6);
   m_shapeColor_Normal := MakeColor(0.9, 0.7, 0.7);

   m_pairColor := MakeColor(0.3, 0.9, 0.9);
   m_aabbColor := MakeColor(0.9, 0.3, 0.9);
   m_world_aabbColor := MakeColor(0.3, 0.9, 0.9);
   m_coreColor := MakeColor(0.9, 0.6, 0.6);
   m_jointLineColor := MakeColor(0.5, 0.8, 0.8);
end;

//////////////////////////////////////////////////////////////
// World

constructor Tb2World.Create(const gravity: TVector2; doSleep: Boolean);
begin
   m_destructionListener := nil;
   m_debugDraw := nil;

   m_bodyList := nil;
   m_jointList := nil;

   m_bodyCount := 0;
   m_jointCount := 0;

   {$IFDEF CONTROLLERS}
   m_controllerList := nil;
   m_controllerCount := 0;
   {$ENDIF}

   m_warmStarting := True;
   m_continuousPhysics := True;

 	 m_subStepping := False;
	 m_stepComplete := True;

   m_allowSleep := doSleep;
   m_gravity := gravity;

   m_flags := e_world_clearForces;
   m_inv_dt0 := 0.0;

   m_contactManager := Tb2ContactManager.Create;
end;

destructor Tb2World.Destroy;
var
   p: Tb2Body;
begin
   // Free all shapes
   while Assigned(m_bodyList) do
   begin
      p := m_bodyList.m_next;
      DestroyBody(m_bodyList);
      m_bodyList := p;
   end;

   m_contactManager.Free;
end;

procedure Tb2World.SetContactFilter(filter: Tb2ContactFilter);
begin
   m_contactManager.m_contactFilter := filter;
end;

procedure Tb2World.SetContactListener(listener: Tb2ContactListener);
begin
   m_contactManager.m_contactListener := listener;
end;

var
   world_solve_island: Tb2Island;
   world_solve_stack: TList;
procedure Tb2World.Solve(const step: Tb2TimeStep);
var
   i: Integer;
   b, seed, other: Tb2Body;
   contact: Pb2Contact;
   j: Tb2Joint;
   stackCount: Int32;
   ce: Pb2ContactEdge;
   je: Pb2JointEdge;
   {$IFDEF CONTROLLERS}
   ctrl: Tb2Controller;
   {$ENDIF}
begin
   {$IFDEF CONTROLLERS}
   // Step all controllers
   ctrl := m_controllerList;
   while Assigned(ctrl) do
   begin
      ctrl.Step(step);
      ctrl := ctrl.m_next;
   end;
   {$ENDIF}

   // Size the island for the worst case.
   world_solve_island.Reset(m_bodyCount, m_contactManager.m_contactCount,
      m_jointCount, m_contactManager.m_contactListener);

   // Clear all the island flags.
   b := m_bodyList;
   while Assigned(b) do
   begin
      b.m_flags := b.m_flags and (not e_body_islandFlag);
      b := b.m_next;
   end;

   contact := m_contactManager.m_contactList;
   while Assigned(contact) do
   begin
      contact^.m_flags := contact^.m_flags and (not e_contact_islandFlag);
      contact := contact^.m_next;
   end;

   j := m_jointList;
   while Assigned(j) do
   begin
      j.m_islandFlag := False;
      j := j.m_next;
   end;

   // Build and simulate all awake islands.
   world_solve_stack.Count := m_bodyCount;
   seed := m_bodyList;
   while Assigned(seed) do
   begin
      if seed.m_flags and e_body_islandFlag <> 0 then
      begin
         seed := seed.m_next;
         Continue;
      end;

      if (not seed.IsAwake) or (not seed.IsActive)  then
      begin
         seed := seed.m_next;
         Continue;
      end;

      // The seed can be dynamic or kinematic.
      if seed.m_type = b2_staticBody then
      begin
         seed := seed.m_next;
         Continue;
      end;

      // Reset island and stack.
      world_solve_island.Clear;
      stackCount := 0;
      world_solve_stack[stackCount] := seed;
      Inc(stackCount);
      seed.m_flags := seed.m_flags or e_body_islandFlag;

      // Perform a depth first search (DFS) on the constraint graph.
      while (stackCount > 0) do
      begin
         // Grab the next body off the stack and add it to the island.
         Dec(stackCount);
         b := Tb2Body(world_solve_stack[stackCount]);
         //b2Assert(b->IsActive() == True);
         world_solve_island.Add(b);

         // Make sure the body is awake.
         b.SetAwake(True);

         // To keep islands as small as possible, we don't
         // propagate islands across static bodies.
         if b.m_type = b2_staticBody then
            Continue;

         // Search all contacts connected to this body.
         ce := b.m_contactList;
         while Assigned(ce) do
         begin
            contact := ce^.contact;

            // Has this contact already been added to an island?
            if contact^.m_flags and e_contact_islandFlag <> 0 then
            begin
               ce := ce^.next;
               Continue;
            end;

            // Is this contact solid and touching?
            {$IFDEF OP_OVERLOAD}
            if (not contact^.IsEnabled) or (not contact^.IsTouching) then
            {$ELSE}
            if (not IsEnabled(contact^)) or (not IsTouching(contact^)) then
            {$ENDIF}
            begin
               ce := ce^.next;
               Continue;
            end;

            // Skip sensors.
            if contact^.m_fixtureA.m_isSensor or contact^.m_fixtureB.m_isSensor then
            begin
               ce := ce^.next;
               Continue;
            end;

            world_solve_island.Add(contact);
            contact^.m_flags := contact^.m_flags or e_contact_islandFlag;

            other := ce^.other;
            // Was the other body already added to this island?
            if (other.m_flags and e_body_islandFlag) <> 0 then
            begin
               ce := ce^.next;
               Continue;
            end;

            //b2Assert(stackCount < stackSize);
            world_solve_stack[stackCount] := other;
            Inc(stackCount);

            other.m_flags := other.m_flags or e_body_islandFlag;
            ce := ce^.next;
         end;

         // Search all joints connect to this body.
         je := b.m_jointList;
         while Assigned(je) do
         begin
            if je^.joint.m_islandFlag then
            begin
               je := je^.next;
               Continue;
            end;

            other := je^.other;
            // Don't simulate joints connected to inactive bodies.
            if not other.IsActive then
            begin
               je := je^.next;
               Continue;
            end;

            world_solve_island.Add(je^.joint);
            je^.joint.m_islandFlag := True;

            if (other.m_flags and e_body_islandFlag) <> 0 then
            begin
               je := je^.next;
               Continue;
            end;

            //b2Assert(stackCount < stackSize);
            world_solve_stack[stackCount] := other;
            Inc(stackCount);
            other.m_flags := other.m_flags or e_body_islandFlag;
            je := je^.next;
         end;
      end;

      world_solve_island.Solve(step, m_gravity, m_allowSleep);

      // Post solve cleanup.
      for i := 0 to world_solve_island.m_bodyCount - 1 do
      begin
         // Allow static bodies to participate in other islands.
         b := Tb2Body(world_solve_island.m_bodies[i]);
         if b.m_type = b2_staticBody then
            b.m_flags := b.m_flags and (not e_body_islandFlag);
      end;
      seed := seed.m_next;
   end;

   // Synchronize fixtures, check for out of range bodies.
   b := m_bodyList;
   while Assigned(b) do
   begin
      // If a body was not in an island then it did not move.
      if b.m_flags and e_body_islandFlag = 0 then
      begin
         b := b.m_next;
         Continue;
      end;

      if b.m_type = b2_staticBody then
      begin
         b := b.m_next;
         Continue;
      end;
      b.SynchronizeFixtures; // Update fixtures (for broad-phase).
      b := b.m_next;
   end;

	 // Look for new contacts.
	 m_contactManager.FindNewContacts;
end;

procedure Tb2World.SolveTOI(const step: Tb2TimeStep);
var
   i: Integer;
   b, bA, bB, body, other: Tb2Body;
   awakeA, awakeB, collideA, collideB: Boolean;
   c, minContact, contact: Pb2Contact;
   minAlpha, alpha, alpha0, beta: Float;
   input: Tb2TOIInput;
   output: Tb2TOIOutput;
   backup, backup1, backup2: Tb2Sweep;
   bodies: array[0..1] of Tb2Body;
   ce: Pb2ContactEdge;
   subStep: Tb2TimeStep;
begin
   world_solve_island.Reset(2 * b2_maxTOIContacts, b2_maxTOIContacts,
      0, m_contactManager.m_contactListener);

   if m_stepComplete then
   begin
      b := m_bodyList;
      while Assigned(b) do
      begin
         b.m_flags := b.m_flags and (not e_body_islandFlag);
         b.m_sweep.alpha0 := 0.0;
         b := b.m_next;
      end;

      c := m_contactManager.m_contactList;
      while Assigned(c) do
      begin
         // Invalidate TOI
         c.m_flags := c.m_flags and (not (e_contact_toiFlag or e_contact_islandFlag));
         c := c.m_next;
      end;
   end;

   // Find TOI events and solve them.
   while True do
   begin
      // Find the first TOI.
      minContact := nil;
      minAlpha := 1.0;

      c := m_contactManager.m_contactList;
      while Assigned(c) do
      begin
         // Is this contact disabled?
         {$IFDEF OP_OVERLOAD}
         if not c.IsEnabled then
         {$ELSE}
         if not IsEnabled(c^) then
         {$ENDIF}
         begin
            c := c.m_next;
            Continue;
         end;

         alpha := 1.0;
         if (c.m_flags and e_contact_toiFlag) <> 0  then
            alpha := c.m_toi // This contact has a valid cached TOI.
         else
         begin
            // Is there a sensor?
            if c.m_fixtureA.IsSensor or c.m_fixtureB.IsSensor then
            begin
               c := c.m_next;
               Continue;
            end;

            bA := c.m_fixtureA.m_body;
            bB := c.m_fixtureB.m_body;

            //b2BodyType typeA := bA.GetType();
            //b2BodyType typeB := bB.GetType();
            //b2Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

            awakeA := bA.IsAwake and (bA.m_type <> b2_staticBody);
            awakeB := bB.IsAwake and (bB.m_type <> b2_staticBody);
            // Is at least one body awake?
            if (not awakeA) and (not awakeB) then
            begin
               c := c.m_next;
               Continue;
            end;

            collideA := bA.IsBullet or (bA.m_type <> b2_dynamicBody);
            collideB := bB.IsBullet or (bB.m_type <> b2_dynamicBody);
            // Are these two non-bullet dynamic bodies?
            if (not collideA) and (not collideB) then
            begin
               c := c.m_next;
               Continue;
            end;

            // Compute the TOI for this contact.
            // Put the sweeps onto the same time interval.
            alpha0 := bA.m_sweep.alpha0;

            if bA.m_sweep.alpha0 < bB.m_sweep.alpha0 then
            begin
               alpha0 := bB.m_sweep.alpha0;
               {$IFDEF OP_OVERLOAD}
               bA.m_sweep.Advance(alpha0);
               {$ELSE}
               Advance(bA.m_sweep, alpha0);
               {$ENDIF}
            end
            else if bB.m_sweep.alpha0 < bA.m_sweep.alpha0 then
            begin
               alpha0 := bA.m_sweep.alpha0;
               {$IFDEF OP_OVERLOAD}
               bB.m_sweep.Advance(alpha0);
               {$ELSE}
               Advance(bB.m_sweep, alpha0);
               {$ENDIF}
            end;

            //b2Assert(alpha0 < 1.0f);
            // Compute the time of impact in interval [0, minTOI]
            {$IFDEF OP_OVERLOAD}
            input.proxyA.SetShape(c.m_fixtureA.m_shape, c.m_indexA);
            input.proxyB.SetShape(c.m_fixtureB.m_shape, c.m_indexB);
            {$ELSE}
            SetShape(input.proxyA, c.m_fixtureA.m_shape, c.m_indexA);
            SetShape(input.proxyB, c.m_fixtureB.m_shape, c.m_indexB);
            {$ENDIF}
            input.sweepA := bA.m_sweep;
            input.sweepB := bB.m_sweep;
            input.tMax := 1.0;

            b2TimeOfImpact(output, input);

            // Beta is the fraction of the remaining portion of the .
            beta := output.t;

            if output.state = e_toi_touching then
               alpha := b2Min(alpha0 + (1.0 - alpha0) * beta, 1.0)
            else
               alpha := 1.0;

            c.m_toi := alpha;
            c.m_flags := c.m_flags or e_contact_toiFlag;
         end;

         if alpha < minAlpha then
         begin
            // This is the minimum TOI found so far.
            minContact := c;
            minAlpha := alpha;
         end;

         c := c.m_next;
      end;

      if (not Assigned(minContact)) or ( 1.0 - 10.0 * FLT_EPSILON < minAlpha) then
      begin
         // No more TOI events. Done!
         m_stepComplete := True;
         Break;
      end;

      // Advance the bodies to the TOI.
      bA := minContact.m_fixtureA.m_body;
      bB := minContact.m_fixtureB.m_body;

      backup1 := bA.m_sweep;
      backup2 := bB.m_sweep;

      bA.Advance(minAlpha);
      bB.Advance(minAlpha);

      // The TOI contact likely has some new contact points.
      {$IFDEF OP_OVERLOAD}
      minContact.Update(m_contactManager.m_contactListener);
      {$ELSE}
      Update(minContact^, m_contactManager.m_contactListener);
      {$ENDIF}
      minContact.m_flags := minContact.m_flags and (not e_contact_toiFlag);

      // Is the contact solid?
      {$IFDEF OP_OVERLOAD}
      if (not minContact.IsEnabled) or (not minContact.IsTouching) then
      {$ELSE}
      if (not IsEnabled(minContact^)) or (not IsTouching(minContact^)) then
      {$ENDIF}
      begin
         // Restore the sweeps.
         {$IFDEF OP_OVERLOAD}
         minContact.SetEnabled(False);
         {$ELSE}
         SetEnabled(minContact^, False);
         {$ENDIF}
         bA.m_sweep := backup1;
         bB.m_sweep := backup2;
         bA.SynchronizeTransform;
         bB.SynchronizeTransform;
         Continue;
      end;

      bA.SetAwake(True);
      bB.SetAwake(True);

      // Build the island
      world_solve_island.Clear;
      world_solve_island.Add(bA);
      world_solve_island.Add(bB);
      world_solve_island.Add(minContact);

      bA.m_flags := bA.m_flags or e_body_islandFlag;
      bB.m_flags := bB.m_flags or e_body_islandFlag;
      minContact.m_flags := minContact.m_flags or e_contact_islandFlag;

      // Get contacts on bodyA and bodyB.
      bodies[0] := bA;
      bodies[1] := bB;
      for i := 0 to 1 do
      begin
         body := bodies[i];
         if body.m_type = b2_dynamicBody then
         begin
            ce := body.m_contactList;
            while Assigned(ce) and (world_solve_island.m_bodyCount < b2_maxTOIContacts) do
            begin
               contact := ce.contact;

               // Has this contact already been added to the island?
               if (contact.m_flags and e_contact_islandFlag) <> 0 then
               begin
                  ce := ce.next;
                  Continue;
               end;

               // Only add static, kinematic, or bullet bodies.
               other := ce.other;
               if (other.m_type = b2_dynamicBody) and (not body.IsBullet) and (not other.IsBullet) then
               begin
                  ce := ce.next;
                  Continue;
               end;

               // Skip sensors.
               if contact.m_fixtureA.m_isSensor or contact.m_fixtureB.m_isSensor then
               begin
                  ce := ce.next;
                  Continue;
               end;

               // Tentatively advance the body to the TOI.
               backup := other.m_sweep;
               if (other.m_flags and e_body_islandFlag) = 0 then
                  other.Advance(minAlpha);

               // Update the contact points
               {$IFDEF OP_OVERLOAD}
               contact.Update(m_contactManager.m_contactListener);
               {$ELSE}
               Update(contact^, m_contactManager.m_contactListener);
               {$ENDIF}

               // Was the contact disabled by the user?
               {$IFDEF OP_OVERLOAD}
               if not contact.IsEnabled then
               {$ELSE}
               if not IsEnabled(contact^) then
               {$ENDIF}
               begin
                  other.m_sweep := backup;
                  other.SynchronizeTransform;
                  ce := ce.next;
                  Continue;
               end;

               // Are there contact points?
               {$IFDEF OP_OVERLOAD}
               if not contact.IsTouching then
               {$ELSE}
               if not IsTouching(contact^) then
               {$ENDIF}
               begin
                  other.m_sweep := backup;
                  other.SynchronizeTransform;
                  ce := ce.next;
                  Continue;
               end;

               // Add the contact to the island
               contact.m_flags := contact.m_flags or e_contact_islandFlag;
               world_solve_island.Add(contact);

               // Has the other body already been added to the island?
               if (other.m_flags and e_body_islandFlag) <> 0 then
               begin
                  ce := ce.next;
                  Continue;
               end;

               // Add the other body to the island.
               other.m_flags := other.m_flags or e_body_islandFlag;

               if other.m_type <> b2_staticBody then
                  other.SetAwake(True);

               world_solve_island.Add(other);
               ce := ce.next;
            end;
         end;
      end;

      subStep.dt := (1.0 - minAlpha) * step.dt;
      subStep.inv_dt := 1.0 / subStep.dt;
      subStep.dtRatio := 1.0;
      subStep.positionIterations := 20;
      subStep.velocityIterations := step.velocityIterations;
      subStep.warmStarting := false;
      world_solve_island.SolveTOI(subStep, bA, bB);

      // Reset island flags and synchronize broad-phase proxies.
      for i := 0 to world_solve_island.m_bodyCount - 1 do
         with Tb2Body(world_solve_island.m_bodies[i]) do
         begin
            m_flags := m_flags and (not e_body_islandFlag);
            if m_type <> b2_dynamicBody then
               Continue;
            SynchronizeFixtures;

            // Invalidate all contact TOIs on this displaced body.
            ce := m_contactList;
            while Assigned(ce) do
            begin
               ce.contact.m_flags := ce.contact.m_flags and (not (e_contact_toiFlag or e_contact_islandFlag));
               ce := ce.next;
            end;
         end;

      // Commit fixture proxy movements to the broad-phase so that new contacts are created.
      // Also, some contacts can be destroyed.
      m_contactManager.FindNewContacts;

      if m_subStepping then
      begin
         m_stepComplete := False;
         Break;
      end;
   end;
end;

procedure Tb2World.ClearForces;
var
   body: Tb2Body;
begin
   body := m_bodyList;
   while Assigned(body) do
      with body do
      begin
         m_force := b2Vec2_Zero;
         m_torque := 0.0;
         body := m_next;
      end;
end;

procedure Tb2World.DrawDebugData;
var
   i: Integer;
   xf: Tb2Transform;
   b: Tb2Body;
   f: Tb2Fixture;
   j: Tb2Joint;
   c: Pb2Contact;
   aabb: Tb2AABB;
   vs: TVectorArray4;
   {$IFDEF CONTROLLERS}ctrl: Tb2Controller;{$ENDIF}
begin
   if not Assigned(m_debugDraw) then
      Exit;

   with m_debugDraw do
   begin
      if e_shapeBit in m_drawFlags then
      begin
         b := m_bodyList;
         while Assigned(b) do
         begin
            f := b.GetFixtureList;
            while Assigned(f) do
            begin
               if not b.IsActive then
                  DrawShape(f, b.m_xf, m_shapeColor_Inactive)
               else if b.m_type = b2_staticBody then
                  DrawShape(f, b.m_xf, m_shapeColor_Static)
               else if b.m_type = b2_kinematicBody then
                  DrawShape(f, b.m_xf, m_shapeColor_kinematic)
               else if not b.IsAwake then
                  DrawShape(f, b.m_xf, m_shapeColor_Sleeping)
               else
                  DrawShape(f, b.m_xf, m_shapeColor_Normal);

               f := f.m_next;
            end;
            b := b.m_next;
         end;
      end;

      if e_jointBit in m_drawFlags then
      begin
         j := m_jointList;
         while Assigned(j) do
         begin
            DrawJoint(j);
            j := j.m_next;
         end;
      end;

      {$IFDEF CONTROLLERS}
      if e_controllerBit in m_drawFlags then
      begin
         ctrl := m_controllerList;
         while Assigned(ctrl) do
         begin
            ctrl.Draw(m_debugDraw);
            ctrl := ctrl.m_next;
         end;
      end;
      {$ENDIF}

      if e_pairBit in m_drawFlags then
      begin
         c := m_contactManager.m_contactList;
         while Assigned(c) do
            with c^ do
            begin
               (*{$IFDEF OP_OVERLOAD}
               DrawSegment(m_fixtureA.m_aabb.GetCenter,
                  m_fixtureB.m_aabb.GetCenter, m_pairColor);
               {$ELSE}
               DrawSegment(GetCenter(m_fixtureA.m_aabb),
                  GetCenter(m_fixtureB.m_aabb), m_pairColor);
               {$ENDIF}
               c := m_next;*)
            end;
      end;

      if e_aabbBit in m_drawFlags then
      begin
         b := m_bodyList;
         while Assigned(b) do
         begin
            if not b.IsActive then
            begin
               b := b.m_next;
               Continue;
            end;

            f := b.GetFixtureList;
            while Assigned(f) do
            begin
               for i := 0 to f.m_proxyCount - 1 do
               begin
					        aabb := m_contactManager.m_broadPhase.GetFatAABB(f.m_proxies[i].proxyId)^;
                  {$IFDEF OP_OVERLOAD}
                  vs[0].SetValue(aabb.lowerBound.x, aabb.lowerBound.y);
                  vs[1].SetValue(aabb.upperBound.x, aabb.lowerBound.y);
                  vs[2].SetValue(aabb.upperBound.x, aabb.upperBound.y);
                  vs[3].SetValue(aabb.lowerBound.x, aabb.upperBound.y);
                  {$ELSE}
                  SetValue(vs[0], aabb.lowerBound.x, aabb.lowerBound.y);
                  SetValue(vs[1], aabb.upperBound.x, aabb.lowerBound.y);
                  SetValue(vs[2], aabb.upperBound.x, aabb.upperBound.y);
                  SetValue(vs[3], aabb.lowerBound.x, aabb.upperBound.y);
                  {$ENDIF}
                  DrawPolygon4(vs, 4, m_aabbColor);
               end;
               f := f.m_next;
            end;
            b := b.m_next;
         end;
      end;

      if e_centerOfMassBit in m_drawFlags then
      begin
         b := m_bodyList;
         while Assigned(b) do
         begin
            xf := b.m_xf;
            xf.position := b.GetWorldCenter;
            DrawTransform(xf);
            b := b.m_next;
         end;
      end;
   end;
end;

type
   Tb2WorldQueryWrapper = class(Tb2GenericCallBackWrapper)
   public
      broadPhase: Tb2BroadPhase;
      callback: Tb2QueryCallback;

      function QueryCallback(proxyId: Int32): Boolean; override;
   end;

   Tb2WorldRayCastWrapper = class(Tb2GenericCallBackWrapper)
   public
      broadPhase: Tb2BroadPhase;
	    callback: Tb2RayCastCallback;

      function RayCastCallback(const input: Tb2RayCastInput; proxyId: Int32): Float; override;
   end;

var
   world_query_wrapper: Tb2WorldQueryWrapper;
   world_raycast_wrapper: Tb2WorldRayCastWrapper;

{ Tb2WorldQueryWrapper }

function Tb2WorldQueryWrapper.QueryCallback(proxyId: Int32): Boolean;
var
   proxy: Pb2FixtureProxy;
begin
	 proxy := Pb2FixtureProxy(broadPhase.GetUserData(proxyId));
	 Result := callback.ReportFixture(proxy^.fixture);
end;

{ Tb2WorldRayCastWrapper }

function Tb2WorldRayCastWrapper.RayCastCallback(const input: Tb2RayCastInput;
   proxyId: Int32): Float;
var
   proxy: Pb2FixtureProxy;
   fixture: Tb2Fixture;
   output: Tb2RayCastOutput;
   fraction: Float;
   point: TVector2;
begin
   proxy := Pb2FixtureProxy(broadPhase.GetUserData(proxyId));
   fixture := proxy^.fixture;
   if fixture.RayCast(output, input, proxy^.childIndex) then
   begin
      fraction := output.fraction;
      {$IFDEF OP_OVERLOAD}
      point := (1.0 - fraction) * input.p1 + fraction * input.p2;
      {$ELSE}
      point := Add(Multiply(input.p1, (1.0 - fraction)), Multiply(input.p2, fraction));
      {$ENDIF}
      Result := callback.ReportFixture(fixture, point, output.normal, fraction);
   end
   else
      Result := input.maxFraction;
end;

procedure Tb2World.QueryAABB(callback: Tb2QueryCallback; const aabb: Tb2AABB);
begin
   world_query_wrapper.broadPhase := m_contactManager.m_broadPhase;
   world_query_wrapper.callback := callback;
   m_contactManager.m_broadPhase.Query(world_query_wrapper, aabb);
end;

procedure Tb2World.RayCast(callback: Tb2RayCastCallback; const point1, point2: TVector2);
var
   input: Tb2RayCastInput;
begin
   world_raycast_wrapper.broadPhase := m_contactManager.m_broadPhase;
   world_raycast_wrapper.callback := callback;
   with input do
   begin
      maxFraction := 1.0;
      p1 := point1;
      p2 := point2;
   end;
   m_contactManager.m_broadPhase.RayCast(world_raycast_wrapper, input);
end;

function Tb2World.GetContactList: Pb2Contact;
begin
   Result := m_contactManager.m_contactList;
end;

function Tb2World.GetContactCount: Int32;
begin
   Result := m_contactManager.m_contactCount;
end;

procedure Tb2World.DrawShape(fixture: Tb2Fixture; const xf: Tb2Transform;
  const color: RGBA);
var
   i: Integer;
   center, v1, v2: TVector2;
   vertices: Tb2PolyVertices;
begin
   with m_debugDraw do
      case fixture.GetType of
         e_circleShape:
            with Tb2CircleShape(fixture.m_shape) do
            begin
               center := b2Mul(xf, m_p);
               m_debugDraw.DrawSolidCircle(center, xf.R.col1, m_radius, color);
            end;
         e_edgeShape:
            with Tb2EdgeShape(fixture.m_shape) do
            begin
               v1 := b2Mul(xf, m_vertex1);
               v2 := b2Mul(xf, m_vertex2);
               m_debugDraw.DrawSegment(v1, v2, color);
            end;
         e_polygonShape:
            with Tb2PolygonShape(fixture.m_shape) do
            begin
               //b2Assert(vertexCount <= b2_maxPolygonVertices);
               for i := 0 to m_vertexCount - 1 do
                  vertices[i] := b2Mul(xf, m_vertices[i]);

               DrawSolidPolygon(vertices, m_vertexCount, color);
            end;
         e_loopShape:
            with Tb2LoopShape(fixture.m_shape) do
            begin
               v1 := b2Mul(xf, m_vertices[m_count - 1]);
               for i := 0 to m_count - 1 do
               begin
                  v2 := b2Mul(xf, m_vertices[i]);
                  m_debugDraw.DrawSegment(v1, v2, color);
                  v1 := v2;
               end;
            end;
      end;
end;

procedure Tb2World.DrawJoint(joint: Tb2Joint);
var
   p1, p2, s1, s2: TVector2;
begin
   p1 := joint.GetAnchorA;
   p2 := joint.GetAnchorB;

   with m_debugDraw do
   begin
      case joint.m_type of
         e_distanceJoint: DrawSegment(p1, p2, m_jointLineColor);
         e_pulleyJoint:
            with Tb2PulleyJoint(joint) do
            begin
               s1 := GetGroundAnchorA;
               s2 := GetGroundAnchorB;
               DrawSegment(s1, p1, m_jointLineColor);
               DrawSegment(s2, p2, m_jointLineColor);
               DrawSegment(s1, s2, m_jointLineColor);
            end;
         e_mouseJoint: ;
         e_fixedJoint: DrawSegment(p1, p2, m_jointLineColor);
      else
         DrawSegment(joint.m_bodyA.m_xf.position, p1, m_jointLineColor);
         DrawSegment(p1, p2, m_jointLineColor);
         DrawSegment(joint.m_bodyB.m_xf.position, p2, m_jointLineColor);
      end;
   end;
end;

function Tb2World.CreateBody(def: Tb2BodyDef; AutoFreeBodyDef: Boolean = True): Tb2Body;
begin
   //b2Assert(IsLocked() == false);
   if IsLocked then
   begin
      Result := nil;
      Exit;
   end;

   Result := Tb2Body.Create(def, Self);

   // Add to world doubly linked list.
   Result.m_prev := nil;
   Result.m_next := m_bodyList;
   if Assigned(m_bodyList) then
      m_bodyList.m_prev := Result;
   m_bodyList := Result;
   Inc(m_bodyCount);

   if AutoFreeBodyDef then
      def.Free;
end;

procedure Tb2World.DestroyBody(body: Tb2Body; DoFree: Boolean = True);
var
   je, je0: Pb2JointEdge;
   ce, ce0: Pb2ContactEdge;
   f, f0: Tb2Fixture;
   {$IFDEF CONTROLLERS}
   coe, coe0: Pb2ControllerEdge;
   {$ENDIF}
begin
   //b2Assert(m_bodyCount > 0);
   //b2Assert(IsLocked() == false);
   if IsLocked then
      Exit;

   // Delete the attached joints.
   je := body.m_jointList;

   if Assigned(m_destructionListener) then
   begin
      while Assigned(je) do
      begin
         je0 := je;
         je := je^.next;
         m_destructionListener.SayGoodbye(je0^.joint);
         DestroyJoint(je0^.joint);
      end;
   end
   else
   begin
      while Assigned(je) do
      begin
         je0 := je;
         je := je^.next;
         DestroyJoint(je0^.joint);
      end;
   end;
   body.m_jointList := nil;

   {$IFDEF CONTROLLERS}
   // Detach controllers attached to this body
   coe := body.m_controllerList;
   while Assigned(coe) do
   begin
      coe0 := coe;
      coe := coe^.nextController;
      coe0^.controller.RemoveBody(body);
   end;
   {$ENDIF}

   // Delete the attached contacts.
   ce := body.m_contactList;
   while Assigned(ce) do
   begin
      ce0 := ce;
      ce := ce^.next;
      m_contactManager.Destroy(ce0^.contact);
   end;
   body.m_contactList := nil;

   // Delete the attached fixtures. This destroys broad-phase proxies.
   f := body.m_fixtureList;
   while Assigned(f) do
   begin
      f0 := f;
      f := f.m_next;

      if Assigned(m_destructionListener) then
         m_destructionListener.SayGoodbye(f0);

      f0.DestroyProxies(m_contactManager.m_broadPhase);
      f0.Free;
   end;
   body.m_fixtureList := nil;
   body.m_fixtureCount := 0;

   // Remove world body list.
   if Assigned(body.m_prev) then
      body.m_prev.m_next := body.m_next;

   if Assigned(body.m_next) then
      body.m_next.m_prev := body.m_prev;

   if body = m_bodyList then
      m_bodyList := body.m_next;

   Dec(m_bodyCount);
   if DoFree then
      body.Destroy2;
end;

function Tb2World.CreateJoint(def: Tb2JointDef; AutoFreeJointDef: Boolean = True): Tb2Joint;
var
   j: Tb2Joint;
   edge: Pb2ContactEdge;
begin
   //b2Assert(IsLocked() == false);
   if IsLocked then
   begin
      Result := nil;
      Exit;
   end;

   Result := nil;
   case def.JointType of
      e_unknownJoint: Exit;
      e_revoluteJoint: j := Tb2RevoluteJoint.Create(Tb2RevoluteJointDef(def));
      e_prismaticJoint: j := Tb2PrismaticJoint.Create(Tb2PrismaticJointDef(def));
      e_distanceJoint: j := Tb2DistanceJoint.Create(Tb2DistanceJointDef(def));
      e_pulleyJoint: j := Tb2PulleyJoint.Create(Tb2PulleyJointDef(def));
      e_mouseJoint: j := Tb2MouseJoint.Create(Tb2MouseJointDef(def));
      e_gearJoint: j := Tb2GearJoint.Create(Tb2GearJointDef(def));
      e_lineJoint: j := Tb2LineJoint.Create(Tb2LineJointDef(def));
      e_weldJoint: j := Tb2WeldJoint.Create(Tb2WeldJointDef(def));
      e_frictionJoint: j := Tb2FrictionJoint.Create(Tb2FrictionJointDef(def));
      e_fixedJoint: j := Tb2FixedJoint.Create(Tb2FixedJointDef(def));
   end;

   // Connect to the world list.
   j.m_prev := nil;
   j.m_next := m_jointList;
   if Assigned(m_jointList) then
      m_jointList.m_prev := j;

   m_jointList := j;
   Inc(m_jointCount);

   // Connect to the bodies' doubly linked lists.
   j.m_edgeA.joint := j;
   j.m_edgeA.other := j.m_bodyB;
   j.m_edgeA.prev := nil;
   j.m_edgeA.next := j.m_bodyA.m_jointList;
   if Assigned(j.m_bodyA.m_jointList) then
      j.m_bodyA.m_jointList.prev := @j.m_edgeA;
   j.m_bodyA.m_jointList := @j.m_edgeA;

   j.m_edgeB.joint := j;
   j.m_edgeB.other := j.m_bodyA;
   j.m_edgeB.prev := nil;
   j.m_edgeB.next := j.m_bodyB.m_jointList;
   if Assigned(j.m_bodyB.m_jointList) then
      j.m_bodyB.m_jointList.prev := @j.m_edgeB;
   j.m_bodyB.m_jointList := @j.m_edgeB;

   // If the joint prevents collisions, then flag any contacts for filtering.
   if not def.collideConnected then
   begin
      edge := def.bodyB.GetContactList;
      while Assigned(edge) do
      begin
         if edge^.other = def.bodyA then
         begin
            // Flag the contact for filtering at the next time step (where either body is awake).
            {$IFDEF OP_OVERLOAD}
            edge^.contact.FlagForFiltering;
            {$ELSE}
            FlagForFiltering(edge^.contact^);
            {$ENDIF}
         end;

         edge := edge^.next;
      end;
   end;

   if AutoFreeJointDef then
      def.Free;

   // Note: creating a joint doesn't wake the bodies.
   Result := j;
end;

procedure Tb2World.DestroyJoint(j: Tb2Joint);
var
   collideConnected: Boolean;
   edge: Pb2ContactEdge;
begin
   //b2Assert(IsLocked() == false);
   if IsLocked then
      Exit;

   collideConnected := j.m_collideConnected;

   // Remove from the doubly linked list.
   if Assigned(j.m_prev) then
      j.m_prev.m_next := j.m_next;

   if Assigned(j.m_next) then
      j.m_next.m_prev := j.m_prev;

   if j = m_jointList then
      m_jointList := j.m_next;

   // Disconnect from island graph.
   // Wake up connected bodies.
   j.m_bodyA.SetAwake(True);
   j.m_bodyB.SetAwake(True);

   // Remove from body 1.
   if Assigned(j.m_edgeA.prev) then
      j.m_edgeA.prev^.next := j.m_edgeA.next;

   if Assigned(j.m_edgeA.next) then
      j.m_edgeA.next^.prev := j.m_edgeA.prev;

   if (@j.m_edgeA) = j.m_bodyA.m_jointList then
      j.m_bodyA.m_jointList := j.m_edgeA.next;

   j.m_edgeA.prev := nil;
   j.m_edgeA.next := nil;

   // Remove from body 2
   if Assigned(j.m_edgeB.prev) then
      j.m_edgeB.prev^.next := j.m_edgeB.next;

   if Assigned(j.m_edgeB.next) then
      j.m_edgeB.next^.prev := j.m_edgeB.prev;

   if (@j.m_edgeB) = j.m_bodyB.m_jointList then
      j.m_bodyB.m_jointList := j.m_edgeB.next;

   j.m_edgeB.prev := nil;
   j.m_edgeB.next := nil;

   //b2Assert(m_jointCount > 0);
   Dec(m_jointCount);

   // If the joint prevents collisions, then flag any contacts for filtering.
   if not collideConnected then
   begin
      edge := j.m_bodyB.GetContactList;
      while Assigned(edge) do
      begin
         if edge^.other = j.m_bodyA then
         begin
            // Flag the contact for filtering at the next time step (where either
            // body is awake).
            {$IFDEF OP_OVERLOAD}
            edge^.contact.FlagForFiltering;
            {$ELSE}
            FlagForFiltering(edge^.contact^);
            {$ENDIF}
         end;
         edge := edge^.next;
      end;
   end;
   j.Free;
end;

{$IFDEF CONTROLLERS}
procedure Tb2World.AddController(c: Tb2Controller);
begin
   c.m_next := m_controllerList;
   c.m_prev := nil;
   m_controllerList := c;
   c.m_world := Self;
   Inc(m_controllerCount);
end;

procedure Tb2World.RemoveController(c: Tb2Controller);
begin
   //TODO: Remove bodies from controller
   if Assigned(c.m_prev) then
      c.m_prev.m_next := c.m_next;
   if Assigned(c.m_next) then
      c.m_next.m_prev := c.m_prev;
   if m_controllerList = c then
      m_controllerList := c.m_next;

   Dec(m_controllerCount);
end;
{$ENDIF}

{$IFDEF COMPUTE_PHYSICSTIME}
function GetRawReferenceTime: Double;
var
   counter: Int64;
begin
   QueryPerformanceCounter(counter);
   Result := counter / vCounterFrequency;
end;
{$ENDIF}

procedure Tb2World.Step(timeStep: Float; velocityIterations,
   positionIterations: Int32; Draw: Boolean = False);
var
   step: Tb2TimeStep;
   {$IFDEF COMPUTE_PHYSICSTIME}old_physicsTime: Double;{$ENDIF}
begin
   {$IFDEF COMPUTE_PHYSICSTIME}
   old_physicsTime := GetRawReferenceTime;
   {$ENDIF}

   // If new fixtures were added, we need to find the new contacts.
   if m_flags and e_world_newFixture <> 0 then
   begin
      m_contactManager.FindNewContacts;
      m_flags := m_flags and (not e_world_newFixture);
   end;

   m_flags := m_flags or e_world_locked;

   step.dt := timeStep;
   step.velocityIterations := velocityIterations;
   step.positionIterations := positionIterations;
   if timeStep > 0.0 then
      step.inv_dt := 1.0 / timeStep
   else
      step.inv_dt := 0.0;

   step.dtRatio := m_inv_dt0 * timeStep;
   step.warmStarting := m_warmStarting;

   // Update contacts. This is where some contacts are destroyed.
   m_contactManager.Collide;

   // Integrate velocities, solve velocity constraints, and integrate positions.
   if timeStep > 0.0 then
   begin
      if m_stepComplete then
         Solve(step);

      // Handle TOI events.
      if m_continuousPhysics then
         SolveTOI(step);

      m_inv_dt0 := step.inv_dt;
   end;

   if m_flags and e_world_clearForces <> 0 then
      ClearForces;

   m_flags := m_flags and (not e_world_locked);

   {$IFDEF COMPUTE_PHYSICSTIME}
   m_physicsTime := GetRawReferenceTime - old_physicsTime;
   {$ENDIF}

   if Draw then
      DrawDebugData;
end;

function Tb2World.GetProxyCount: Int32;
begin
   Result := m_contactManager.m_broadPhase.GetProxyCount;
end;

procedure Tb2World.SetGravity(const gravity: TVector2);
begin
   m_gravity := gravity;
end;

procedure Tb2World.WakeAllSleepingBodies;
var
   b: Tb2Body;
begin
   b := m_bodyList;
   while Assigned(b) do
   begin
      if b.IsActive and (b.m_type = b2_dynamicBody) and (not b.IsAwake) then
         b.SetAwake(True);
      b := b.m_next;
   end;
end;

function Tb2World.IsLocked: Boolean;
begin
   Result := (m_flags and e_world_locked) = e_world_locked;
end;

procedure Tb2World.SetAutoClearForces(flag: Boolean);
begin
   if flag then
      m_flags := m_flags or e_world_clearForces
	 else
      m_flags := m_flags and (not e_world_clearForces);
end;

function Tb2World.GetAutoClearForces: Boolean;
begin
   Result := (m_flags and e_world_clearForces) = e_world_clearForces;
end;

////////////////////////////////////////////////////
// Contact

{ Tb2Contact }

{$IFDEF OP_OVERLOAD}
procedure Tb2Contact.Update(listener: Tb2ContactListener);
var
   i, j: Integer;
   oldManifold: Tb2Manifold;
   touching, wasTouching, sensor: Boolean;
   bodyA, bodyB: Tb2Body;
   mp1, mp2: Pb2ManifoldPoint;
   id2key: UInt32;
   found: Boolean;
begin
   oldManifold := m_manifold;
   m_flags := m_flags or e_contact_enabledFlag; // Re-enable this contact.

   //touching := False;
   wasTouching := (m_flags and e_contact_touchingFlag) = e_contact_touchingFlag;

   sensor := m_fixtureA.IsSensor or m_fixtureB.IsSensor;

   bodyA := m_fixtureA.m_body;
   bodyB := m_fixtureB.m_body;

   // Is this contact a sensor?
   if sensor then
   begin
      touching := b2TestOverlap(m_fixtureA.m_shape, m_fixtureB.m_shape,
         m_indexA, m_indexB, bodyA.m_xf, bodyB.m_xf);

      // Sensors don't generate manifolds.
      m_manifold.pointCount := 0;
   end
   else
   begin
      m_evaluateProc(@Self, m_manifold, m_fixtureA, m_fixtureB, bodyA.m_xf, bodyB.m_xf, True);
      touching := m_manifold.pointCount > 0;

      // Match old contact ids to new contact ids and copy the
      // stored impulses to warm start the solver.
      for i := 0 to m_manifold.pointCount - 1 do
      begin
         mp2 := @m_manifold.points[i];
         mp2^.normalImpulse := 0.0;
         mp2^.tangentImpulse := 0.0;
         id2key := mp2^.id.key;
         found := False;

         for j := 0 to oldManifold.pointCount - 1 do
         begin
            mp1 := @oldManifold.points[j];

            if mp1^.id.key = id2key then
            begin
               mp2^.normalImpulse := mp1^.normalImpulse;
               mp2^.tangentImpulse := mp1^.tangentImpulse;
               found := True;
               Break;
            end;
         end;

         if not found then
         begin
            mp2^.normalImpulse := 0.0;
            mp2^.tangentImpulse := 0.0;
         end;
      end;

      if touching xor wasTouching then
      begin
         bodyA.SetAwake(True);
         bodyB.SetAwake(True);
      end;
   end;

   if touching then
      m_flags := m_flags or e_contact_touchingFlag
   else
      m_flags := m_flags and (not e_contact_touchingFlag);

   if Assigned(listener) then
   begin
      if (not wasTouching) and touching then
         listener.BeginContact(Self);

      if wasTouching and (not touching) then
         listener.EndContact(Self);

      if (not sensor) and touching then
         listener.PreSolve(Self, oldManifold);
   end;
end;

function Tb2Contact.GetManifold: Pb2Manifold;
begin
   Result := @m_manifold;
end;

procedure Tb2Contact.GetWorldManifold(var worldManifold: Tb2WorldManifold);
begin
   {$IFDEF OP_OVERLOAD}
   worldManifold.Initialize(m_manifold, m_fixtureA.m_body.m_xf,
      m_fixtureB.m_body.m_xf, m_fixtureA.m_shape.m_radius, m_fixtureB.m_shape.m_radius);
   {$ELSE}
   Initialize(worldManifold, m_manifold, m_fixtureA.m_body.m_xf,
      m_fixtureB.m_body.m_xf, m_fixtureA.m_shape.m_radius, m_fixtureB.m_shape.m_radius);
   {$ENDIF}
end;

procedure Tb2Contact.FlagForFiltering;
begin
   m_flags := m_flags or e_contact_filterFlag;
end;

function Tb2Contact.IsTouching: Boolean;
begin
   Result := (m_flags and e_contact_touchingFlag) = e_contact_touchingFlag;
end;

procedure Tb2Contact.SetEnabled(flag: Boolean);
begin
   if flag then
      m_flags := m_flags or e_contact_enabledFlag
   else
      m_flags := m_flags and (not e_contact_enabledFlag);
end;

function Tb2Contact.IsEnabled: Boolean;
begin
   Result := (m_flags and e_contact_enabledFlag) = e_contact_enabledFlag;
end;
{$ENDIF}

{ Tb2ContactFilter }

function Tb2ContactFilter.ShouldCollide(fixtureA, fixtureB: Tb2Fixture): Boolean;
begin
   with fixtureA.m_filter do
      if (groupIndex = fixtureB.m_filter.groupIndex) and (groupIndex <> 0) then
         Result := groupIndex > 0
      else
         Result := ((maskBits and fixtureB.m_filter.categoryBits) <> 0) and
            ((categoryBits and fixtureB.m_filter.maskBits) <> 0);
end;

{ Tb2ContactListener }

procedure Tb2ContactListener.BeginContact(var contact: Tb2Contact);
begin
end;

procedure Tb2ContactListener.EndContact(var contact: Tb2Contact);
begin
end;

procedure Tb2ContactListener.PreSolve(var contact: Tb2Contact; const oldManifold: Tb2Manifold);
begin
end;

procedure Tb2ContactListener.PostSolve(var contact: Tb2Contact; const impulse: Tb2ContactImpulse);
begin
end;

{ Tb2ContactSolver }

destructor Tb2ContactSolver.Destroy;
begin
   if Assigned(m_constraints) then
      FreeMemory(m_constraints);
end;

procedure Tb2ContactSolver.Initialize(contacts: TList; count: Int32; impulseRatio: Float;
   warmStarting: Boolean);
var
   i, j: Integer;
   cc: Pb2ContactConstraint;
   cp: Pb2ManifoldPoint;
begin
   m_count := count;
   if Assigned(m_constraints) then
      FreeMemory(m_constraints);
   m_constraints := Pb2ContactConstraint(GetMemory(m_count * SizeOf(Tb2ContactConstraint)));

   // Initialize position independent portions of the constraints.
   for i := 0 to m_count - 1 do
      with Pb2Contact(contacts[i])^ do
      begin
         //b2Assert(manifold.pointCount > 0);
         cc := m_constraints;
         Inc(cc, i);
         with cc^ do
         begin
            friction := b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
            restitution := b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
            manifold := @m_manifold;
            bodyA := m_fixtureA.m_body;
            bodyB := m_fixtureB.m_body;
            normal := b2Vec2_Zero;
            radiusA := m_fixtureA.m_shape.m_radius;
            radiusB := m_fixtureB.m_shape.m_radius;
            pointCount := m_manifold.pointCount;
            localNormal := m_manifold.localNormal;
            localPoint := m_manifold.localPoint;
            manifoldType := m_manifold.manifoldType;
         end;

         for j := 0 to cc^.pointCount - 1 do
         begin
            cp := @m_manifold.points[j];
            FillChar(cc.points[j], SizeOf(Tb2ContactConstraintPoint), 0);
            with cc.points[j] do
            begin
               if warmStarting then
               begin
                  normalImpulse := impulseRatio * cp^.normalImpulse;
                  tangentImpulse := impulseRatio * cp^.tangentImpulse;
               end;
               localPoint := cp^.localPoint;
            end;
         end;

         {$IFDEF OP_OVERLOAD}
         cc^.K.SetZero;
         cc^.normalMass.SetZero;
         {$ELSE}
         SetZero(cc^.K);
         SetZero(cc^.normalMass);
         {$ENDIF}
      end;
end;

// Initialize position dependent portions of the velocity constraints.
procedure Tb2ContactSolver.InitializeVelocityConstraints;
const
   k_maxConditionNumber = 100.0;
var
   i, j: Integer;
   cc: Pb2ContactConstraint;
   bodyA, bodyB: Tb2Body;
   manifold: Pb2Manifold;
   vA, vB, tangent: TVector2;
   wA, wB, radiusA, radiusB, rA, rB, kNormal, kTangent, vRel: Float;
   worldManifold: Tb2WorldManifold;
   cp: Pb2ManifoldPoint;
   ccp: Pb2ContactConstraintPoint;
   invMass, invIA, invIB, k11, k12, k22, rn1A, rn1B, rn2A, rn2B: Float;
begin
   for i := 0 to m_count - 1 do
   begin
      cc := m_constraints;
      Inc(cc, i);

      bodyA := cc.bodyA;
      bodyB := cc.bodyB;
      manifold := cc.manifold;

      vA := bodyA.m_linearVelocity;
      vB := bodyB.m_linearVelocity;
      wA := bodyA.m_angularVelocity;
      wB := bodyB.m_angularVelocity;

      //b2Assert(manifold->pointCount > 0);
      {$IFDEF OP_OVERLOAD}
      worldManifold.Initialize(manifold^, bodyA.m_xf, bodyB.m_xf, cc.radiusA, cc.radiusB);
      {$ELSE}
      UPhysics2D.Initialize(worldManifold, manifold^, bodyA.m_xf, bodyB.m_xf, cc.radiusA, cc.radiusB);
      {$ENDIF}

      cc.normal := worldManifold.normal;

      for j := 0 to cc^.pointCount - 1 do
      begin
         cp := @manifold^.points[j];
         ccp := @cc.points[j];

         with ccp^ do
         begin
            {$IFDEF OP_OVERLOAD}
            rA := worldManifold.points[j] - bodyA.m_sweep.c;
            rB := worldManifold.points[j] - bodyB.m_sweep.c;
            {$ELSE}
            rA := Subtract(worldManifold.points[j], bodyA.m_sweep.c);
            rB := Subtract(worldManifold.points[j], bodyB.m_sweep.c);
            {$ENDIF}
         end;

         rA := Sqr(b2Cross(ccp^.rA, cc^.normal));
         rB := Sqr(b2Cross(ccp^.rB, cc^.normal));
         kNormal := bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rA + bodyB.m_invI * rB;

         //b2Assert(kNormal > b2_epsilon);
         ccp^.normalMass := 1.0 / kNormal;

         tangent := b2Cross(cc.normal, 1.0);

         rA := Sqr(b2Cross(ccp^.rA, tangent));
         rB := Sqr(b2Cross(ccp^.rB, tangent));
         kTangent := bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rA + bodyB.m_invI * rB;

         //b2Assert(kTangent > b2_epsilon);
         ccp^.tangentMass := 1.0 /  kTangent;

         // Setup a velocity bias for restitution.
         ccp^.velocityBias := 0.0;
         {$IFDEF OP_OVERLOAD}
         vRel := b2Dot(cc.normal, vB + b2Cross(wB, ccp^.rB) - vA - b2Cross(wA, ccp^.rA));
         {$ELSE}
         vRel := b2Dot(cc.normal, Subtract(Add(vB, b2Cross(wB, ccp^.rB)), Add(vA, b2Cross(wA, ccp^.rA))));
         {$ENDIF}
         if vRel < -b2_velocityThreshold then
            ccp^.velocityBias := -cc.restitution * vRel;
      end;

      // If we have two points, then prepare the block solver.
      if cc^.pointCount = 2 then
      begin
         invMass := bodyA.m_invMass + bodyB.m_invMass;
         invIA := bodyA.m_invI;
         invIB := bodyB.m_invI;

         with cc.points[0] do
         begin
            rn1A := b2Cross(rA, cc^.normal);
            rn1B := b2Cross(rB, cc^.normal);
         end;
         with cc.points[1] do
         begin
            rn2A := b2Cross(rA, cc^.normal);
            rn2B := b2Cross(rB, cc^.normal);
         end;

         k11 := invMass + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
         k22 := invMass + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
         k12 := invMass + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

         // Ensure a reasonable condition number.
         if k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12) then
         begin
            with cc^ do
            begin
               // K is safe to invert.
               {$IFDEF OP_OVERLOAD}
               K.col1.SetValue(k11, k12);
               K.col2.SetValue(k12, k22);
               normalMass := K.GetInverse;
               {$ELSE}
               SetValue(K.col1, k11, k12);
               SetValue(K.col2, k12, k22);
               normalMass := GetInverse(K);
               {$ENDIF}
            end;
         end
         else
         begin
            // The constraints are redundant, just use one.
            // TODO_ERIN use deepest?
            cc^.pointCount := 1;
         end;
      end;
   end;
end;

procedure Tb2ContactSolver.WarmStart;
var
   i, j: Integer;
   c: Pb2ContactConstraint;
   ccp: Pb2ContactConstraintPoint;
   P, tangent: TVector2;
begin
   // Warm start.
   for i := 0 to m_count - 1 do
   begin
      c := m_constraints;
      Inc(c, i);
      with c^ do
      begin
         tangent := b2Cross(normal, 1.0);
         for j := 0 to c^.pointCount - 1 do
         begin
            ccp := @c^.points[j];
            {$IFDEF OP_OVERLOAD}
            P := ccp^.normalImpulse * normal + ccp^.tangentImpulse * tangent;
            {$ELSE}
            P := Add(Multiply(normal, ccp^.normalImpulse), Multiply(tangent, ccp^.tangentImpulse));
            {$ENDIF}
            bodyA.m_angularVelocity := bodyA.m_angularVelocity - bodyA.m_invI * b2Cross(ccp^.rA, P);
            bodyB.m_angularVelocity := bodyB.m_angularVelocity + bodyB.m_invI * b2Cross(ccp^.rB, P);

            {$IFDEF OP_OVERLOAD}
            bodyA.m_linearVelocity.SubtractBy(bodyA.m_invMass * P);
            bodyB.m_linearVelocity.AddBy(bodyB.m_invMass * P);
            {$ELSE}
            SubtractBy(bodyA.m_linearVelocity, Multiply(P, bodyA.m_invMass));
            AddBy(bodyB.m_linearVelocity, Multiply(P, bodyB.m_invMass));
            {$ENDIF}
         end;
      end;
   end;
end;

procedure Tb2ContactSolver.SolveVelocityConstraints;
var
   i, j: Integer;
   c: Pb2ContactConstraint;
   cp1, cp2: Pb2ContactConstraintPoint;
   bodyA, bodyB: Tb2Body;
   wA, wB, invMassA, invMassB, invIA, invIB: Float;
   friction, lambda, maxFriction, newImpulse: Float;
   vn1, vn2: Float;
   vA, vB, dv: TVector2;
   normal, tangent, P, a, b, x, d, P1, P2, dv1, dv2: TVector2;
begin
   for i := 0 to m_count - 1 do
   begin
      c := m_constraints;
      Inc(c, i);

      bodyA := c^.bodyA;
      bodyB := c^.bodyB;
      with bodyA do
      begin
         wA := m_angularVelocity;
         vA := m_linearVelocity;
         invMassA := m_invMass;
         invIA := m_invI;
      end;

      with bodyB do
      begin
         wB := m_angularVelocity;
         vB := m_linearVelocity;
         invMassB := m_invMass;
         invIB := m_invI;
      end;

      normal := c^.normal;
      tangent := b2Cross(normal, 1.0);
      friction := c^.friction;

      //b2Assert(c.pointCount == 1 || c.pointCount == 2);
      // Solve tangent constraints
      for j := 0 to c^.pointCount - 1 do
         with c.points[j] do
         begin
            // Relative velocity at contact
            {$IFDEF OP_OVERLOAD}
            dv := vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA);
            {$ELSE}
            dv := Subtract(Add(vB, b2Cross(wB, rB)), Add(vA, b2Cross(wA, rA)));
            {$ENDIF}

            // Compute tangent force
            lambda := tangentMass * (-b2Dot(dv, tangent));

            // b2Clamp the accumulated force
            maxFriction := friction * normalImpulse;
            newImpulse := b2Clamp(tangentImpulse + lambda, -maxFriction, maxFriction);
            lambda := newImpulse - tangentImpulse;

            // Apply contact impulse
            {$IFDEF OP_OVERLOAD}
            P := lambda * tangent;
            vA.SubtractBy(invMassA * P);
            vB.AddBy(invMassB * P);
            {$ELSE}
            P := Multiply(tangent, lambda);
            SubtractBy(vA, Multiply(P, invMassA));
            AddBy(vB, Multiply(P, invMassB));
            {$ENDIF}
            wA := wA - invIA * b2Cross(rA, P);
            wB := wB + invIB * b2Cross(rB, P);

            tangentImpulse := newImpulse;
         end;

      // Solve normal constraints
      if c^.pointCount = 1 then
      begin
         with c.points[0] do
         begin
            // Relative velocity at contact
            {$IFDEF OP_OVERLOAD}
            dv := vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA);
            {$ELSE}
            dv := Subtract(Add(vB, b2Cross(wB, rB)), Add(vA, b2Cross(wA, rA)));
            {$ENDIF}

            // Compute normal impulse
            lambda := -normalMass * (b2Dot(dv, normal) - velocityBias);

            // b2Clamp the accumulated impulse
            newImpulse := b2Max(normalImpulse + lambda, 0.0);
            lambda := newImpulse - normalImpulse;

            // Apply contact impulse
            {$IFDEF OP_OVERLOAD}
            P := lambda * normal;
            vA.SubtractBy(invMassA * P);
            vB.AddBy(invMassB * P);
            {$ELSE}
            P := Multiply(normal, lambda);
            SubtractBy(vA, Multiply(P, invMassA));
            AddBy(vB, Multiply(P, invMassB));
            {$ENDIF}
            wA := wA - invIA * b2Cross(rA, P);
            wB := wB + invIB * b2Cross(rB, P);

            normalImpulse := newImpulse;
         end;
      end
      else
      begin
         // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
         // Build the mini LCP for this contact patch
         //
         // vn := A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i := 0 with i := 1..2
         //
         // A := J * W * JT and J := ( -n, -r1 x n, n, r2 x n )
         // b := vn_0 - velocityBias
         //
         // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
         // implies that we must have in any solution either vn_i := 0 or x_i := 0. So for the 2D contact problem the cases
         // vn1 := 0 and vn2 := 0, x1 := 0 and x2 := 0, x1 := 0 and vn2 := 0, x2 := 0 and vn1 := 0 need to be tested. The first valid
         // solution that satisfies the problem is chosen.
         //
         // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
         // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
         //
         // Substitute:
         //
         // x := x' - a
         //
         // Plug into above equation:
         //
         // vn := A * x + b
         //    := A * (x' - a) + b
         //    := A * x' + b - A * a
         //    := A * x' + b'
         // b' := b - A * a;

         cp1 := @c.points[0];
         cp2 := @c.points[1];

         SetValue(a, cp1^.normalImpulse, cp2^.normalImpulse);
         //b2Assert(a.x >= 0.0f && a.y >= 0.0f);

         // Relative velocity at contact
         {$IFDEF OP_OVERLOAD}
         dv1 := vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
         dv2 := vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
         {$ELSE}
         dv1 := Subtract(Add(vB, b2Cross(wB, cp1.rB)), Add(vA, b2Cross(wA, cp1.rA)));
         dv2 := Subtract(Add(vB, b2Cross(wB, cp2.rB)), Add(vA, b2Cross(wA, cp2.rA)));
         {$ENDIF}

         // Compute normal velocity
         vn1 := b2Dot(dv1, normal);
         vn2 := b2Dot(dv2, normal);

         b.x := vn1 - cp1.velocityBias;
         b.y := vn2 - cp2.velocityBias;
         {$IFDEF OP_OVERLOAD}
         b.SubtractBy(b2Mul(c.K, a));
         {$ELSE}
         SubtractBy(b, b2Mul(c.K, a));
         {$ENDIF}
         while True do
         begin
            //
            // Case 1: vn := 0
            //
            // 0 := A * x' + b'
            //
            // Solve for x':
            //
            // x' := - inv(A) * b'
            //
            {$IFDEF OP_OVERLOAD}
            x := -b2Mul(c.normalMass, b);
            {$ELSE}
            x := Negative(b2Mul(c.normalMass, b));
            {$ENDIF}

            if (x.x >= 0.0) and (x.y >= 0.0) then
            begin
               // Resubstitute for the incremental impulse
               {$IFDEF OP_OVERLOAD}
               d := x - a;
               // Apply incremental impulse
               P1 := d.x * normal;
               P2 := d.y * normal;
               vA.SubtractBy(invMassA * (P1 + P2));
               vB.AddBy(invMassB * (P1 + P2));
               {$ELSE}
               d := Subtract(x, a);
               // Apply incremental impulse
               P1 := Multiply(normal, d.x);
               P2 := Multiply(normal, d.y);
               SubtractBy(vA, Multiply(Add(P1, P2), invMassA));
               AddBy(vB, Multiply(Add(P1, P2), invMassB));
               {$ENDIF}

               wA := wA - (invIA * (b2Cross(cp1^.rA, P1) + b2Cross(cp2^.rA, P2)));
               wB := wB + invIB * (b2Cross(cp1^.rB, P1) + b2Cross(cp2^.rB, P2));

               // Accumulate
               cp1^.normalImpulse := x.x;
               cp2^.normalImpulse := x.y;
               Break;
            end;

            //
            // Case 2: vn1 := 0 and x2 := 0
            //
            //   0 := a11 * x1' + a12 * 0 + b1'
            // vn2 := a21 * x1' + a22 * 0 + b2'
            //
            x.x := -cp1.normalMass * b.x;
            x.y := 0.0;
            //vn1 := 0.0;
            vn2 := c.K.col1.y * x.x + b.y;

            if (x.x >= 0.0) and (vn2 >= 0.0) then
            begin
               // Resubstitute for the incremental impulse
               {$IFDEF OP_OVERLOAD}
               d := x - a;
               // Apply incremental impulse
               P1 := d.x * normal;
               P2 := d.y * normal;
               vA.SubtractBy(invMassA * (P1 + P2));
               vB.AddBy(invMassB * (P1 + P2));
               {$ELSE}
               d := Subtract(x, a);
               // Apply incremental impulse
               P1 := Multiply(normal, d.x);
               P2 := Multiply(normal, d.y);
               SubtractBy(vA, Multiply(Add(P1, P2), invMassA));
               AddBy(vB, Multiply(Add(P1, P2), invMassB));
               {$ENDIF}

               wA := wA - (invIA * (b2Cross(cp1^.rA, P1) + b2Cross(cp2^.rA, P2)));
               wB := wB + invIB * (b2Cross(cp1^.rB, P1) + b2Cross(cp2^.rB, P2));

               // Accumulate
               cp1^.normalImpulse := x.x;
               cp2^.normalImpulse := x.y;
               Break;
            end;

            //
            // Case 3: vn2 := 0 and x1 := 0
            //
            // vn1 := a11 * 0 + a12 * x2' + b1'
            //   0 := a21 * 0 + a22 * x2' + b2'
            //
            x.x := 0.0;
            x.y := -cp2.normalMass * b.y;
            vn1 := c.K.col2.x * x.y + b.x;
            //vn2 := 0.0;

            if (x.y >= 0.0) and (vn1 >= 0.0) then
            begin
               // Resubstitute for the incremental impulse
               {$IFDEF OP_OVERLOAD}
               d := x - a;
               // Apply incremental impulse
               P1 := d.x * normal;
               P2 := d.y * normal;
               vA.SubtractBy(invMassA * (P1 + P2));
               vB.AddBy(invMassB * (P1 + P2));
               {$ELSE}
               d := Subtract(x, a);
               // Apply incremental impulse
               P1 := Multiply(normal, d.x);
               P2 := Multiply(normal, d.y);
               SubtractBy(vA, Multiply(Add(P1, P2), invMassA));
               AddBy(vB, Multiply(Add(P1, P2), invMassB));
               {$ENDIF}

               wA := wA - (invIA * (b2Cross(cp1^.rA, P1) + b2Cross(cp2^.rA, P2)));
               wB := wB + invIB * (b2Cross(cp1^.rB, P1) + b2Cross(cp2^.rB, P2));

               // Accumulate
               cp1^.normalImpulse := x.x;
               cp2^.normalImpulse := x.y;
               Break;
            end;

            //
            // Case 4: x1 := 0 and x2 := 0
            //
            // vn1 := b1
            // vn2 := b2;
            x.x := 0.0;
            x.y := 0.0;
            vn1 := b.x;
            vn2 := b.y;

            if (vn1 >= 0.0) and (vn2 >= 0.0) then
            begin
               // Resubstitute for the incremental impulse
               {$IFDEF OP_OVERLOAD}
               d := x - a;
               // Apply incremental impulse
               P1 := d.x * normal;
               P2 := d.y * normal;
               vA.SubtractBy(invMassA * (P1 + P2));
               vB.AddBy(invMassB * (P1 + P2));
               {$ELSE}
               d := Subtract(x, a);
               // Apply incremental impulse
               P1 := Multiply(normal, d.x);
               P2 := Multiply(normal, d.y);
               SubtractBy(vA, Multiply(Add(P1, P2), invMassA));
               AddBy(vB, Multiply(Add(P1, P2), invMassB));
               {$ENDIF}

               wA := wA - (invIA * (b2Cross(cp1^.rA, P1) + b2Cross(cp2^.rA, P2)));
               wB := wB + invIB * (b2Cross(cp1^.rB, P1) + b2Cross(cp2^.rB, P2));

               // Accumulate
               cp1^.normalImpulse := x.x;
               cp2^.normalImpulse := x.y;
               Break;
            end;
            Break; // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
         end;
      end;

      bodyA.m_linearVelocity := vA;
      bodyA.m_angularVelocity := wA;
      bodyB.m_linearVelocity := vB;
      bodyB.m_angularVelocity := wB;
   end;
end;

procedure Tb2ContactSolver.StoreImpulses;
var
   i, j: Integer;
   m: Pb2Manifold;
   c: Pb2ContactConstraint;
begin
   for i := 0 to m_count - 1 do
   begin
      c := m_constraints;
      Inc(c, i);

      m := c^.manifold;
      for j := 0 to c^.pointCount - 1 do
      begin
         m^.points[j].normalImpulse := c^.points[j].normalImpulse;
         m^.points[j].tangentImpulse := c^.points[j].tangentImpulse;
      end;
   end;
end;

type
   Tb2PositionSolverManifold = class
   public
      normal, point: TVector2;
      separation: Float;

      procedure Initialize(const cc: Tb2ContactConstraint; index: Int32);
   end;

{ Tb2PositionSolverManifold }

procedure Tb2PositionSolverManifold.Initialize(const cc: Tb2ContactConstraint; index: Int32);
var
   pointA, pointB, planePoint: TVector2;
begin
   //b2Assert(cc.pointCount > 0);

   case cc.manifoldType of
      e_manifold_circles:
         begin
            pointA := cc.bodyA.GetWorldPoint(cc.localPoint);
            pointB := cc.bodyB.GetWorldPoint(cc.points[0].localPoint);
            if b2DistanceSquared(pointA, pointB) > FLT_EPSILON * FLT_EPSILON then
            begin
               {$IFDEF OP_OVERLOAD}
               normal := pointB - pointA;
               normal.Normalize;
               {$ELSE}
               normal := Subtract(pointB, pointA);
               Normalize(normal);
               {$ENDIF}
            end
            else
            begin
               normal.x := 1.0;
               normal.y := 0.0;
            end;
            point := b2MiddlePoint(pointA, pointB);
            {$IFDEF OP_OVERLOAD}
            separation := b2Dot(pointB - pointA, normal) - cc.radiusA - cc.radiusB;
            {$ELSE}
            separation := b2Dot(Subtract(pointB, pointA), normal) - cc.radiusA - cc.radiusB;
            {$ENDIF}
         end;
      e_manifold_faceA:
         begin
            normal := cc.bodyA.GetWorldVector(cc.localNormal);
            planePoint := cc.bodyA.GetWorldPoint(cc.localPoint);
            point := cc.bodyB.GetWorldPoint(cc.points[index].localPoint);
            {$IFDEF OP_OVERLOAD}
            separation := b2Dot(point - planePoint, normal) - cc.radiusA - cc.radiusB;
            {$ELSE}
            separation := b2Dot(Subtract(point, planePoint), normal) - cc.radiusA - cc.radiusB;
            {$ENDIF}
         end;
      e_manifold_faceB:
         begin
            normal := cc.bodyB.GetWorldVector(cc.localNormal);
            planePoint := cc.bodyB.GetWorldPoint(cc.localPoint);

            point := cc.bodyA.GetWorldPoint(cc.points[index].localPoint);
            {$IFDEF OP_OVERLOAD}
            separation := b2Dot(point - planePoint, normal) - cc.radiusA - cc.radiusB;
            normal := -normal; // Ensure normal points from A to B
            {$ELSE}
            separation := b2Dot(Subtract(point, planePoint), normal) - cc.radiusA - cc.radiusB;
            normal := Negative(normal); // Ensure normal points from A to B
            {$ENDIF}
         end;
   end;
end;

var
   contactsolver_positionsolver: Tb2PositionSolverManifold;
function Tb2ContactSolver.SolvePositionConstraints(baumgarte: Float): Boolean;
var
   c: Pb2ContactConstraint;
   minSeparation: Float;
   i, j: Integer;
   invMassA, invMassB, invIA, invIB, rnA, rnB: Float;
   _normal, _point, P: TVector2;
   _separation, _c, _K, impulse: Float;
   rA, rB: TVector2;
begin
   minSeparation := 0.0;
   for i := 0 to m_count - 1 do
   begin
      c := m_constraints;
      Inc(c, i);
      with c^ do
      begin
         with bodyA do
         begin
            invMassA := m_mass * m_invMass;
            invIA := m_mass * m_invI;
         end;
         with bodyB do
         begin
            invMassB := m_mass * m_invMass;
            invIB := m_mass * m_invI;
         end;

         // Solve normal constraints
         for j := 0 to pointCount - 1 do
         begin
            with contactsolver_positionsolver do
            begin
               Initialize(c^, j);
               _normal := normal;
               _point := point;
               _separation := separation;
            end;

            {$IFDEF OP_OVERLOAD}
            rA := _point - bodyA.m_sweep.c;
            rB := _point - bodyB.m_sweep.c;
            {$ELSE}
            rA := Subtract(_point, bodyA.m_sweep.c);
            rB := Subtract(_point, bodyB.m_sweep.c);
            {$ENDIF}

            // Track max constraint error.
            minSeparation := b2Min(minSeparation, _separation);

            // Prevent large corrections and allow slop.
            _c := b2Clamp(baumgarte * (_separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0);

            // Compute the effective mass.
            rnA := b2Cross(rA, _normal);
            rnB := b2Cross(rB, _normal);
            _K := invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;

            // Compute normal impulse
            if  _K > 0.0 then
               impulse := -_c / _K
            else
               impulse := 0.0;

            {$IFDEF OP_OVERLOAD}
            P := impulse * _normal;
            bodyA.m_sweep.c.SubtractBy(invMassA * P);
            bodyB.m_sweep.c.AddBy(invMassB * P);
            {$ELSE}
            P := Multiply(_normal, impulse);
            SubtractBy(bodyA.m_sweep.c, Multiply(P, invMassA));
            AddBy(bodyB.m_sweep.c, Multiply(P, invMassB));
            {$ENDIF}

            bodyA.m_sweep.a := bodyA.m_sweep.a - invIA * b2Cross(rA, P);
            bodyB.m_sweep.a := bodyB.m_sweep.a + invIB * b2Cross(rB, P);

            bodyA.SynchronizeTransform;
            bodyB.SynchronizeTransform;
         end;
      end;
   end;

   // We can't expect minSpeparation >= -b2_linearSlop because we don't
   // push the separation above -b2_linearSlop.
   Result := minSeparation >= -1.5 * b2_linearSlop;
end;

// Sequential position solver for position constraints.
function Tb2ContactSolver.SolveTOIPositionConstraints(baumgarte: Float; toiBodyA, toiBodyB: Tb2Body): Boolean;
var
   minSeparation: Float;
   i, j: Integer;
   c: Pb2ContactConstraint;
   massA, massB, invMassA, invIA, invMassB, invIB, separation, _C, _K,
      rnA, rnB, K, impulse: Float;
   normal, point, rA, rB, P: TVector2;
begin
   minSeparation := 0.0;
   for i := 0 to m_count - 1 do
   begin
      c := m_constraints;
      Inc(c, i);
      with c^ do
      begin
         massA := 0.0;
         if (bodyA = toiBodyA) or (bodyA = toiBodyB) then
            massA := bodyA.m_mass;

         massB := 0.0;
         if (bodyB = toiBodyA) or (bodyB = toiBodyB) then
            massB := bodyB.m_mass;

         with bodyA do
         begin
            invMassA := m_mass * m_invMass;
            invIA := m_mass * m_invI;
         end;

         with bodyB do
         begin
            invMassB := m_mass * m_invMass;
            invIB := m_mass * m_invI;
         end;

         // Solve normal constraints
         for j := 0 to c.pointCount - 1 do
         begin
            contactsolver_positionsolver.Initialize(c^, j);
            normal := contactsolver_positionsolver.normal;
            point := contactsolver_positionsolver.point;
            separation := contactsolver_positionsolver.separation;

            {$IFDEF OP_OVERLOAD}
            rA := point - bodyA.m_sweep.c;
            rB := point - bodyB.m_sweep.c;
            {$ELSE}
            rA := Subtract(point, bodyA.m_sweep.c);
            rB := Subtract(point, bodyB.m_sweep.c);
            {$ENDIF}

            // Track max constraint error.
            minSeparation := b2Min(minSeparation, separation);

            // Prevent large corrections and allow slop.
            _C := b2Clamp(baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0);

            // Compute the effective mass.
            rnA := b2Cross(rA, normal);
            rnB := b2Cross(rB, normal);
            _K := invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;

            // Compute normal impulse
            if _K > 0.0 then
               impulse := - _C / _K
            else
               impulse := 0.0;

            {$IFDEF OP_OVERLOAD}
            P := impulse * normal;
            {$ELSE}
            P := Multiply(normal, impulse);
            {$ENDIF}

            with bodyA do
            begin
               {$IFDEF OP_OVERLOAD}
               m_sweep.c.SubtractBy(invMassA * P);
               {$ELSE}
               SubtractBy(m_sweep.c, Multiply(P, invMassA));
               {$ENDIF}
               m_sweep.a := m_sweep.a - invIA * b2Cross(rA, P);
               SynchronizeTransform;
            end;

            with bodyB do
            begin
               {$IFDEF OP_OVERLOAD}
               m_sweep.c.AddBy(invMassB * P);
               {$ELSE}
               AddBy(m_sweep.c, Multiply(P, invMassB));
               {$ENDIF}
               m_sweep.a := m_sweep.a + invIB * b2Cross(rB, P);
               SynchronizeTransform;
            end;
         end;
      end;
   end;

   // We can't expect minSpeparation >= -b2_linearSlop because we don't
   // push the separation above -b2_linearSlop.
   Result := minSeparation >= -1.5 * b2_linearSlop;
end;

{ Tb2ContactManager }

constructor Tb2ContactManager.Create;
begin
   m_broadPhase := Tb2BroadPhase.Create;
   m_contactList := nil;
   m_contactCount := 0;
   m_contactFilter := b2_defaultFilter;
   m_contactListener := b2_defaultListener;
end;

destructor Tb2ContactManager.Destroy;
begin
   m_broadPhase.Free;
end;

procedure Tb2ContactManager.AddPair(proxyUserDataA, proxyUserDataB: Pointer);
var
   proxyA, proxyB: Pb2FixtureProxy;
   indexA, indexB: Int32;
   fixtureA, fixtureB: Tb2Fixture;
   bodyA, bodyB: Tb2Body;
   edge: Pb2ContactEdge;
   c: Pb2Contact;
begin
	 proxyA := Pb2FixtureProxy(proxyUserDataA);
	 proxyB := Pb2FixtureProxy(proxyUserDataB);

	 fixtureA := proxyA^.fixture;
	 fixtureB := proxyB^.fixture;

	 indexA := proxyA^.childIndex;
	 indexB := proxyB^.childIndex;

   bodyA := fixtureA.m_body;
   bodyB := fixtureB.m_body;

   if bodyA = bodyB then // Are the fixtures on the same body?
      Exit;

   // Does a contact already exist?
   edge := bodyB.GetContactList;
   while Assigned(edge) do
   begin
      if edge.other = bodyA then
      begin
         with edge^.contact^ do
         begin
            if ((m_fixtureA = fixtureA) and (m_fixtureB = fixtureB) and
               (m_indexA = indexA) and (m_indexB = indexB)) or
               ((m_fixtureA = fixtureB) and (m_fixtureB = fixtureA) and
               (m_indexA = indexB) and (m_indexB = indexA)) then // A contact already exists.
               Exit;
         end;
      end;

      edge := edge^.next;
   end;

   // Does a joint override collision? Is at least one body dynamic?
   if not bodyB.ShouldCollide(bodyA) then
      Exit;

   // Check user filtering.
   if Assigned(m_contactFilter) and (not m_contactFilter.ShouldCollide(fixtureA, fixtureB)) then
      Exit;

   // Call the factory.
   c := NewContact(fixtureA, fixtureB, indexA, indexB);

   with c^ do
   begin
      // Contact creation may swap fixtures.
      indexA := m_indexA;
      indexB := m_indexB;
      bodyA := m_fixtureA.m_body;
      bodyB := m_fixtureB.m_body;

      // Insert into the world.
      m_prev := nil;
      m_next := m_contactList;
      if Assigned(m_contactList) then
         m_contactList.m_prev := c;
      m_contactList := c;

      // Connect to island graph.
      // Connect to body A
      with m_nodeA do
      begin
         contact := c;
         other := bodyB;
         prev := nil;
         next := bodyA.m_contactList;
         if Assigned(bodyA.m_contactList) then
            bodyA.m_contactList.prev := @m_nodeA;
         bodyA.m_contactList := @m_nodeA;
      end;

      // Connect to body B
      with m_nodeB do
      begin
         contact := c;
         other := bodyA;
         prev := nil;
         next := bodyB.m_contactList;
         if Assigned(bodyB.m_contactList) then
            bodyB.m_contactList.prev := @m_nodeB;
         bodyB.m_contactList := @m_nodeB;
      end;
   end;

   Inc(m_contactCount);
end;

procedure Tb2ContactManager.FindNewContacts;
begin
   m_broadPhase.UpdatePairs(Self);
end;

procedure Tb2ContactManager.Destroy(pc: Pb2Contact);
var
   bodyA, bodyB: Tb2Body;
begin
   with pc^ do
   begin
      bodyA := m_fixtureA.m_body;
      bodyB := m_fixtureB.m_body;

      {$IFDEF OP_OVERLOAD}
      if Assigned(m_contactListener) and IsTouching then
      {$ELSE}
      if Assigned(m_contactListener) and IsTouching(pc^) then
      {$ENDIF}
         m_contactListener.EndContact(pc^);

      // Remove from the world.
      if Assigned(m_prev) then
         m_prev.m_next := m_next;

      if Assigned(m_next) then
         m_next.m_prev := m_prev;

      if pc = m_contactList then
         m_contactList := m_next;

      // Remove from body 1
      if Assigned(m_nodeA.prev) then
         m_nodeA.prev^.next := m_nodeA.next;

      if Assigned(m_nodeA.next) then
         m_nodeA.next^.prev := m_nodeA.prev;

      if (@m_nodeA = bodyA.m_contactList) then
         bodyA.m_contactList := m_nodeA.next;

      // Remove from body 2
      if Assigned(m_nodeB.prev) then
         m_nodeB.prev^.next := m_nodeB.next;

      if Assigned(m_nodeB.next) then
         m_nodeB.next^.prev := m_nodeB.prev;

      if (@m_nodeB = bodyB.m_contactList) then
         bodyB.m_contactList := m_nodeB.next;

      // Call the factory.
      if m_manifold.pointCount > 0 then
      begin
         m_fixtureA.m_body.SetAwake(True);
         m_fixtureB.m_body.SetAwake(True);
      end;
   end;
   FreeContact(pc);
   Dec(m_contactCount);
end;

procedure Tb2ContactManager.Collide;
var
   c, cNuke: Pb2Contact;
   bodyA, bodyB: Tb2Body;
   proxyIdA, proxyIdB: Int32;
   fixtureA, fixtureB: Tb2Fixture;
   overlap: Boolean;
begin
   // Update awake contacts.
   c := m_contactList;
   while Assigned(c) do
   begin
      fixtureA := c^.m_fixtureA;
      fixtureB := c^.m_fixtureB;
      bodyA := fixtureA.m_body;
      bodyB := fixtureB.m_body;

      if (not bodyA.IsAwake) and (not bodyB.IsAwake) then
      begin
         c := c^.m_next;
         Continue;
      end;

      // Is this contact flagged for filtering?
      if (c^.m_flags and e_contact_filterFlag) <> 0 then
      begin
         // Should these bodies collide?
         if not bodyB.ShouldCollide(bodyA) then
         begin
            cNuke := c;
            c := cNuke^.m_next;
            Destroy(cNuke);
            Continue;
         end;

         // Check user filtering.
         if Assigned(m_contactFilter) and (not
            m_contactFilter.ShouldCollide(fixtureA, fixtureB)) then
         begin
            cNuke := c;
            c := cNuke^.m_next;
            Destroy(cNuke);
            Continue;
         end;

         // Clear the filtering flag.
         c^.m_flags := c^.m_flags and (not e_contact_filterFlag);
      end;

		  proxyIdA := fixtureA.m_proxies[c^.m_indexA].proxyId;
		  proxyIdB := fixtureB.m_proxies[c^.m_indexB].proxyId;
      overlap := m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

      // Here we destroy contacts that cease to overlap in the broad-phase.
      if not overlap then
      begin
         cNuke := c;
         c := cNuke^.m_next;
         Destroy(cNuke);
         Continue;
      end;

      // The contact persists.
      {$IFDEF OP_OVERLOAD}
      c^.Update(m_contactListener);
      {$ELSE}
      Update(c^, m_contactListener);
      {$ENDIF}
      c := c^.m_next;
   end;
end;

////////////////////////////////////////////////////
// Island
{ Tb2Island }

(*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and False bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effectivprocedure Tb2Island.Solve(const step: Tb2TimeStep; const gravity: TVector2;
  correctPositions, allowSleep: Boolean);
begin

end;

procedure Tb2Island.SolveTOI(const subStep: Tb2TimeStep);
begin

end;
e mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*)

(*Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*)

constructor Tb2Island.Create;
begin
   m_bodies := TList.Create;
   m_contacts := TList.Create;
   m_joints := TList.Create;
end;

destructor Tb2Island.Destroy;
begin
   // Warning: the order should reverse the constructor order.
   m_joints.Free;
   m_contacts.Free;
   m_bodies.Free;
end;

procedure Tb2Island.Reset(bodyCapacity, contactCapacity, jointCapacity: Int32;
   listener: Tb2ContactListener);
begin
   m_bodyCapacity := bodyCapacity;
   m_contactCapacity := contactCapacity;
   m_jointCapacity := jointCapacity;
   m_bodyCount := 0;
   m_contactCount := 0;
   m_jointCount := 0;

   m_listener := listener;

   m_bodies.Count := bodyCapacity;
   m_contacts.Count := contactCapacity;
   m_joints.Count := jointCapacity;

   SetLength(m_velocities, m_bodyCapacity);
   SetLength(m_positions, m_bodyCapacity);
end;

procedure Tb2Island.Clear;
begin
   m_bodyCount := 0;
   m_contactCount := 0;
   m_jointCount := 0;
end;

var
   island_solve_contact_solver: Tb2ContactSolver;
procedure Tb2Island.Solve(const step: Tb2TimeStep; const gravity: TVector2;
   allowSleep: Boolean);
const
   linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
   angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;
var
   pswap: Pointer;
   i, j: Integer;
   contactsOkay, jointsOkay: Boolean;
   minSleepTime, ratio, rotation: Float;
   translation: TVector2;
begin
   // Integrate velocities and apply damping.
   for i := 0 to m_bodyCount - 1 do
      with Tb2Body(m_bodies[i]) do
      begin
         if m_type <> b2_dynamicBody then
            Continue;

         // Integrate velocities.
         {$IFDEF OP_OVERLOAD}         
         m_linearVelocity.AddBy(step.dt * (gravity + m_invMass * m_force));
         {$ELSE}
         AddBy(m_linearVelocity, Multiply(UPhysics2DTypes.Add(gravity,
            Multiply(m_force, m_invMass)), step.dt));
         {$ENDIF}                  
         m_angularVelocity := m_angularVelocity + step.dt * m_invI * m_torque;

         // Apply damping.
         // ODE: dv/dt + c * v = 0
         // Solution: v(t) = v0 * exp(-c * t)
         // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
         // v2 = exp(-c * dt) * v1
         // Taylor expansion:
         // v2 = (1.0f - c * dt) * v1
         {$IFDEF OP_OVERLOAD}
         m_linearVelocity.MultiplyBy(b2Clamp(1.0 - step.dt * m_linearDamping, 0.0, 1.0));
         {$ELSE}
         MultiplyBy(m_linearVelocity, b2Clamp(1.0 - step.dt * m_linearDamping, 0.0, 1.0));
         {$ENDIF}
         m_angularVelocity := m_angularVelocity * b2Clamp(1.0 - step.dt *
            m_angularDamping, 0.0, 1.0);
      end;

   // Partition contacts so that contacts with static bodies are solved last.
   j := -1;
   for i := 0 to m_contactCount - 1 do
      with Pb2Contact(m_contacts[i])^ do
         if (m_fixtureA.m_body.m_type <> b2_staticBody) and
            (m_fixtureB.m_body.m_type <> b2_staticBody) then
         begin
            Inc(j);
            pswap := m_contacts[i];
            m_contacts[i] := m_contacts[j];
            m_contacts[j] := pswap;
         end;

   // Initialize velocity constraints.
   island_solve_contact_solver.Initialize(m_contacts, m_contactCount, step.dtRatio, step.warmStarting);
   island_solve_contact_solver.InitializeVelocityConstraints;
   if step.warmStarting then
      island_solve_contact_solver.WarmStart;

   for i := 0 to m_jointCount - 1 do
      Tb2Joint(m_joints[i]).InitVelocityConstraints(step);

   // Solve velocity constraints.
   if (island_solve_contact_solver.m_count > 0) or (m_jointCount > 0) then
      for i := 0 to step.velocityIterations - 1 do
      begin
         for j := 0 to m_jointCount - 1 do
            Tb2Joint(m_joints[j]).SolveVelocityConstraints(step);
         island_solve_contact_solver.SolveVelocityConstraints;
      end;

   // Post-solve (store impulses for warm starting).
   island_solve_contact_solver.StoreImpulses;

   // Integrate positions.
   for i := 0 to m_bodyCount - 1 do
   begin
      with Tb2Body(m_bodies[i]) do
      begin
         if m_type = b2_staticBody then
            Continue;

         // Check for large velocities.
         {$IFDEF OP_OVERLOAD}
         translation := step.dt * m_linearVelocity;
         {$ELSE}
         translation := Multiply(m_linearVelocity, step.dt);
         {$ENDIF}
         if b2Dot(translation, translation) > b2_maxTranslationSquared then
         begin
            {$IFDEF OP_OVERLOAD}
            ratio := b2_maxTranslation / translation.Length;
            m_linearVelocity.MultiplyBy(ratio);
            {$ELSE}
            ratio := b2_maxTranslation / LengthVec(translation);
            MultiplyBy(m_linearVelocity, ratio);
            {$ENDIF}
         end;

         rotation := step.dt * m_angularVelocity;
         if rotation * rotation > b2_maxRotationSquared then
         begin
            ratio := b2_maxRotation / Abs(rotation);
            m_angularVelocity := m_angularVelocity * ratio;
         end;

         // Store positions for continuous collision.
         m_sweep.c0 := m_sweep.c;
         m_sweep.a0 := m_sweep.a;

         // Integrate
         {$IFDEF OP_OVERLOAD}
         m_sweep.c.AddBy(step.dt * m_linearVelocity);
         {$ELSE}
         AddBy(m_sweep.c, Multiply(m_linearVelocity, step.dt));
         {$ENDIF}
         m_sweep.a := m_sweep.a + step.dt * m_angularVelocity;

         // Compute new transform
         SynchronizeTransform;
         // Note: shapes are synchronized later.
      end;
   end;

   // Iterate over constraints.
   for i := 0 to step.positionIterations - 1 do
   begin
      contactsOkay := island_solve_contact_solver.SolvePositionConstraints(b2_contactBaumgarte);
      jointsOkay := True;

      for j := 0 to m_jointCount - 1 do
         jointsOkay := Tb2Joint(m_joints[j]).SolvePositionConstraints(b2_contactBaumgarte) and jointsOkay;

      if contactsOkay and jointsOkay then // Exit early if the position errors are small.
         Break;
   end;

   Report(island_solve_contact_solver.m_constraints);

   if allowSleep then
   begin
      minSleepTime := FLT_MAX;

      for i := 0 to m_bodyCount - 1 do
         with Tb2Body(m_bodies[i]) do
         begin
            if m_type = b2_staticBody then
               Continue;

            if ((m_flags and e_body_autoSleepFlag) = 0) or
               (m_angularVelocity * m_angularVelocity > angTolSqr) or
               (b2Dot(m_linearVelocity, m_linearVelocity) > linTolSqr) then
            begin
               m_sleepTime := 0.0;
               minSleepTime := 0.0;
            end
            else
            begin
               m_sleepTime := m_sleepTime + step.dt;
               minSleepTime := b2Min(minSleepTime, m_sleepTime);
            end;
         end;

      if minSleepTime >= b2_timeToSleep then
         for i := 0 to m_bodyCount - 1 do
            Tb2Body(m_bodies[i]).SetAwake(False);
   end;
end;

procedure Tb2Island.SolveTOI(const subStep: Tb2TimeStep; bodyA, bodyB: Tb2Body);
const
   k_toiBaumgarte = 0.75;
var
   i: Integer;
   b: Tb2Body;
   translation: TVector2;
   rotation: Float;
begin
   island_solve_contact_solver.Initialize(m_contacts, m_contactCount,
      subStep.dtRatio, subStep.warmStarting);

   // Solve position constraints.
   for i := 0 to subStep.positionIterations - 1 do
   begin
      if island_solve_contact_solver.SolveTOIPositionConstraints(k_toiBaumgarte, bodyA, bodyB) then
         Break;
      //if i = subStep.positionIterations - 1 then
   end;

   // Advance bodies to new safe spot
   // Leap of faith to new safe state.
   for i := 0 to m_bodyCount - 1 do
      with Tb2Body(m_bodies[i]).m_sweep do
      begin
         a0 := a;
         c0 := c;
      end;

   // No warm starting is needed for TOI events because warm
   // starting impulses were applied in the discrete solver.
   island_solve_contact_solver.InitializeVelocityConstraints;

   // Solve velocity constraints.
   for i := 0 to subStep.velocityIterations - 1 do
      island_solve_contact_solver.SolveVelocityConstraints;

   // Don't store the TOI contact forces for warm starting
   // because they can be quite large.

   // Integrate positions.
   for i := 0 to m_bodyCount - 1 do
   begin
      b := Tb2Body(m_bodies[i]);
      with b do
      begin
         if m_type = b2_staticBody then
            Continue;

         // Check for large velocities.
         {$IFDEF OP_OVERLOAD}
         translation := subStep.dt * m_linearVelocity;
         {$ELSE}
         translation := Multiply(m_linearVelocity, subStep.dt);
         {$ENDIF}
         if b2Dot(translation, translation) > b2_maxTranslationSquared then
         begin
            {$IFDEF OP_OVERLOAD}
            translation.Normalize;
            m_linearVelocity := (b2_maxTranslation * subStep.inv_dt) * translation;
            {$ELSE}
            Normalize(translation);
            m_linearVelocity := Multiply(translation, b2_maxTranslation * subStep.inv_dt);
            {$ENDIF}
         end;

         rotation := subStep.dt * m_angularVelocity;
         if rotation * rotation > b2_maxRotationSquared then
            if rotation < 0.0 then
               m_angularVelocity := -subStep.inv_dt * b2_maxRotation
            else
               m_angularVelocity := subStep.inv_dt * b2_maxRotation;

         // Integrate

         {$IFDEF OP_OVERLOAD}
         m_sweep.c.AddBy(subStep.dt * m_linearVelocity);
         {$ELSE}
         AddBy(m_sweep.c, Multiply(m_linearVelocity, subStep.dt));
         {$ENDIF}
         m_sweep.a := m_sweep.a + subStep.dt * m_angularVelocity;

         // Compute new transform
         SynchronizeTransform;

         // Note: shapes are synchronized later.
      end;
   end;

   Report(island_solve_contact_solver.m_constraints);
end;

procedure Tb2Island.Add(body: Tb2Body);
begin
	 //b2Assert(m_bodyCount < m_bodyCapacity);
   body.m_islandIndex := m_bodyCount;
   m_bodies[m_bodyCount] := body;
   Inc(m_bodyCount);
end;

procedure Tb2Island.Add(contact: Pb2Contact);
begin
	 //b2Assert(m_contactCount < m_contactCapacity);
   m_contacts[m_contactCount] := contact;
   Inc(m_contactCount);
end;

procedure Tb2Island.Add(joint: Tb2Joint);
begin
	 //b2Assert(m_jointCount < m_jointCapacity);
   m_joints[m_jointCount] := joint;
   Inc(m_jointCount);
end;

procedure Tb2Island.Report(constraints: Pb2ContactConstraint);
var
   i, j: Integer;
   c: Pb2Contact;
   cc: Pb2ContactConstraint;
   impulse: Tb2ContactImpulse;
begin
   if not Assigned(m_listener) then
      Exit;

   for i := 0 to m_contactCount - 1 do
   begin
      c := Pb2Contact(m_contacts[i]);
      cc := constraints;
      Inc(cc, i);

      for j := 0 to cc^.pointCount - 1 do
      begin
        impulse.normalImpulses[j] := cc^.points[j].normalImpulse;
        impulse.tangentImpulses[j] := cc^.points[j].tangentImpulse;
      end;

      m_listener.PostSolve(c^, impulse);
   end;
end;

{ Tb2GrowableStack }

type
   /// This is a growable LIFO stack with an initial capacity of N.
   /// If the stack size exceeds the initial capacity, the heap is used
   /// to increase the size of the stack.
   Tb2GrowableStack = class
   private
      m_stack: PInt32;
      m_count, m_capacity: Int32;
   public
      constructor Create;
      destructor Destroy; override;

      procedure Reset;
      procedure Push(const element: Int32);
      function Pop: Int32;

      property GetCount: Int32 read m_count write m_count;
   end;

const
   _defaultStackCapacity = 256;
var
   _StackArray: array[0.._defaultStackCapacity - 1] of Int32;
   _GrowableStack: Tb2GrowableStack;

constructor Tb2GrowableStack.Create;
begin
   m_stack := @_StackArray[0];
	 m_count := 0;
	 m_capacity := _defaultStackCapacity;
end;

destructor Tb2GrowableStack.Destroy;
begin
   Reset;
   inherited;
end;

procedure Tb2GrowableStack.Reset;
begin
   if m_stack <> @_StackArray[0] then
      FreeMemory(m_stack);
   m_stack := @_StackArray[0];
	 m_count := 0;
	 m_capacity := _defaultStackCapacity;
end;

procedure Tb2GrowableStack.Push(const element: Int32);
var
   old: PInt32;
begin
   if m_count = m_capacity then
   begin
      old := m_stack;
      m_capacity := m_capacity * 2;
      m_stack := PInt32(GetMemory(m_capacity * sizeof(Int32)));
      CopyMemory(m_stack, old, m_count * sizeof(Int32));
      if old <> @_StackArray[0] then
         FreeMemory(old);
   end;

   PInt32(Integer(m_stack) + SizeOf(Int32) * m_count)^ := element;
   Inc(m_count);
end;

function Tb2GrowableStack.Pop: Int32;
begin
   //b2Assert(m_count > 0);
	 Dec(m_count);
	 Result := PInt32(Integer(m_stack) + SizeOf(Int32) * m_count)^;
end;

{ Tb2DynamicTreeNode }
{$IFDEF OP_OVERLOAD}
function Tb2DynamicTreeNode.IsLeaf: Boolean;
begin
   Result := child1 = b2_nullNode;
end;
{$ELSE}
function IsLeaf(const node: Tb2DynamicTreeNode): Boolean; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
begin
   Result := node.child1 = b2_nullNode;
end;
{$ENDIF}

{ Tb2DynamicTree }

constructor Tb2DynamicTree.Create;
begin
   m_root := b2_nullNode;

   m_nodeCount := 0;
   SetCapacity(16);
   m_freeList := 0;

   m_path := 0;
   m_insertionCount := 0;
end;

procedure Tb2DynamicTree.SetCapacity(value: Int32);
var
   i: Integer;
begin
   SetLength(m_nodes, value);

   // Build a linked list for the free list.
   for i := m_nodeCount to value - 2 do
      m_nodes[i].next := i + 1;

   m_nodes[value - 1].next := b2_nullNode;
   m_nodeCapacity := value;
end;

function Tb2DynamicTree.AllocateNode: Int32;
var
   nodeId: Int32;
begin
   // Expand the node pool as needed.
   if m_freeList = b2_nullNode then
   begin
      SetCapacity(m_nodeCapacity * 2);
      m_freeList := m_nodeCount;
   end;

   // Peel a node off the free list.
   nodeId := m_freeList;
   m_freeList := m_nodes[nodeId].next;
   m_nodes[nodeId].parent := b2_nullNode;
   m_nodes[nodeId].child1 := b2_nullNode;
   m_nodes[nodeId].child2 := b2_nullNode;
   m_nodes[nodeId].leafCount := 0;
   Inc(m_nodeCount);
   Result := nodeId;
end;

procedure Tb2DynamicTree.FreeNode(nodeId: Int32);
begin
   //b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
   //b2Assert(0 < m_nodeCount);
   m_nodes[nodeId].next := m_freeList;
   m_freeList := nodeId;
   Dec(m_nodeCount);
end;

procedure Tb2DynamicTree.InsertLeaf(leaf: Int32);
var
   leafAABB, parentAABB, aabb: Tb2AABB;
   siblingArea, parentArea, cost1, cost2, cost3, inheritanceCost: Float;
   sibling, _child1, _child2, oldParent, newParent: Int32;
begin
   Inc(m_insertionCount);

   if m_root = b2_nullNode then
   begin
      m_root := leaf;
      m_nodes[m_root].parent := b2_nullNode;
      Exit;
   end;

   // Find the best sibling for this node
   leafAABB := m_nodes[leaf].aabb;
   sibling := m_root;
   {$IFDEF OP_OVERLOAD}
   while not m_nodes[sibling].IsLeaf do
   {$ELSE}
   while not IsLeaf(m_nodes[sibling]) do
   {$ENDIF}
   begin
      // Expand the node's AABB.
      with m_nodes[sibling] do
      begin
         {$IFDEF OP_OVERLOAD}
         aabb.Combine(leafAABB);
         {$ELSE}
         Combine(aabb, leafAABB);
         {$ENDIF}
         Inc(leafCount);

         _child1 := child1;
         _child2 := child2;

         {$IFDEF OP_OVERLOAD}
         siblingArea := aabb.GetPerimeter;
         parentAABB.Combine(aabb, leafAABB);
         {$ELSE}
         siblingArea := GetPerimeter(aabb);
         Combine(parentAABB, aabb, leafAABB);
         {$ENDIF}
      end;

      {$IFDEF OP_OVERLOAD}
      parentArea := parentAABB.GetPerimeter;
      {$ELSE}
      parentArea := GetPerimeter(parentAABB);
      {$ENDIF}
      cost1 := 2.0 * parentArea;

      inheritanceCost := 2.0 * (parentArea - siblingArea);

      {$IFDEF OP_OVERLOAD}
      if m_nodes[_child1].IsLeaf then
      begin
         aabb.Combine(leafAABB, m_nodes[_child1].aabb);
         cost2 := aabb.GetPerimeter + inheritanceCost;
      end
      else
      begin
         aabb.Combine(leafAABB, m_nodes[_child1].aabb);
         cost2 := aabb.GetPerimeter - m_nodes[_child1].aabb.GetPerimeter + inheritanceCost;
      end;
      if m_nodes[_child2].IsLeaf then
      begin
         aabb.Combine(leafAABB, m_nodes[_child2].aabb);
         cost3 := aabb.GetPerimeter + inheritanceCost;
      end
      else
      begin
         aabb.Combine(leafAABB, m_nodes[_child2].aabb);
         cost3 := aabb.GetPerimeter - m_nodes[_child2].aabb.GetPerimeter + inheritanceCost;
      end;
      {$ELSE}
      if IsLeaf(m_nodes[_child1]) then
      begin
         Combine(aabb, leafAABB, m_nodes[_child1].aabb);
         cost2 := GetPerimeter(aabb) + inheritanceCost;
      end
      else
      begin
         Combine(aabb, leafAABB, m_nodes[_child1].aabb);
         cost2 := GetPerimeter(aabb) - GetPerimeter(m_nodes[_child1].aabb) + inheritanceCost;
      end;
      if IsLeaf(m_nodes[_child2]) then
      begin
         Combine(aabb, leafAABB, m_nodes[_child2].aabb);
         cost3 := GetPerimeter(aabb) + inheritanceCost;
      end
      else
      begin
         Combine(aabb, leafAABB, m_nodes[_child2].aabb);
         cost3 := GetPerimeter(aabb) - GetPerimeter(m_nodes[_child2].aabb) + inheritanceCost;
      end;
      {$ENDIF}

      // Descend according to the minimum cost.
      if (cost1 < cost2) and (cost1 < cost3) then
         Break;

      // Expand the node's AABB to account for the new leaf.
      {$IFDEF OP_OVERLOAD}
      m_nodes[sibling].aabb.Combine(leafAABB);
      {$ELSE}
      Combine(m_nodes[sibling].aabb, leafAABB);
      {$ENDIF}

      // Descend
      if cost2 < cost3 then
         sibling := _child1
      else
         sibling := _child2;
   end;

   // Create a new parent for the siblings.
   oldParent := m_nodes[sibling].parent;
   newParent := AllocateNode;
   with m_nodes[newParent] do
   begin
      parent := oldParent;
      userData := nil;
      {$IFDEF OP_OVERLOAD}
      aabb.Combine(leafAABB, m_nodes[sibling].aabb);
      {$ELSE}
      Combine(aabb, leafAABB, m_nodes[sibling].aabb);
      {$ENDIF}
      leafCount := m_nodes[sibling].leafCount + 1;
   end;

   if oldParent <> b2_nullNode then
   begin
      // The sibling was not the root.
      if m_nodes[oldParent].child1 = sibling then
         m_nodes[oldParent].child1 := newParent
      else
         m_nodes[oldParent].child2 := newParent;

      m_nodes[newParent].child1 := sibling;
      m_nodes[newParent].child2 := leaf;
      m_nodes[sibling].parent := newParent;
      m_nodes[leaf].parent := newParent;
   end
   else
   begin
      // The sibling was the root.
      m_nodes[newParent].child1 := sibling;
      m_nodes[newParent].child2 := leaf;
      m_nodes[sibling].parent := newParent;
      m_nodes[leaf].parent := newParent;
      m_root := newParent;
   end;
end;

procedure Tb2DynamicTree.RemoveLeaf(leaf: Int32);
var
   grandParent, _parent, sibling: Int32;
   oldAABB: Tb2AABB;
begin
   if leaf = m_root then
   begin
      m_root := b2_nullNode;
      Exit;
   end;

   _parent := m_nodes[leaf].parent;
   with m_nodes[_parent] do
   begin
      grandParent := parent;
      if child1 = leaf then
         sibling := child2
      else
         sibling := child1;
   end;

   if grandParent <> b2_nullNode then
   begin
      // Destroy parent and connect sibling to grandParent.
      with m_nodes[grandParent] do
         if child1 = _parent then
            child1 := sibling
         else
            child2 := sibling;
      m_nodes[sibling].parent := grandParent;
      FreeNode(_parent);

      // Adjust ancestor bounds.
      _parent := grandParent;
      while _parent <> b2_nullNode do
         with m_nodes[_parent] do
         begin
            oldAABB := aabb;
            {$IFDEF OP_OVERLOAD}
            aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
            {$ELSE}
            Combine(aabb, m_nodes[child1].aabb, m_nodes[child2].aabb);
            {$ENDIF}

            //b2Assert(m_nodes[parent].leafCount > 0);
            Dec(leafCount);
            _parent := parent;
         end;
   end
   else
   begin
      m_root := sibling;
      m_nodes[sibling].parent := b2_nullNode;
      FreeNode(_parent);
   end;
end;

function Tb2DynamicTree.ComputeHeight(nodeId: Int32): Int32;
var
   height1, height2: Int32;
begin
   if nodeId = b2_nullNode then
   begin
      Result := 0;
      Exit;
   end;

   //b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
   with m_nodes[nodeId] do
   begin
      height1 := ComputeHeight(child1);
      height2 := ComputeHeight(child2);
   end;
   Result := 1 + b2Max(height1, height2);
end;

function Tb2DynamicTree.CountLeaves(nodeId: Int32): Int32;
begin
   if nodeId = b2_nullNode then
   begin
      Result := 0;
      Exit;
   end;

   //b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
   {$IFDEF OP_OVERLOAD}
   if m_nodes[nodeId].IsLeaf then
   {$ELSE}
   if IsLeaf(m_nodes[nodeId]) then
   {$ENDIF}
   begin
      //b2Assert(node->leafCount == 1);
      Result := 1;
      Exit;
   end;

   Result := CountLeaves(m_nodes[nodeId].child1) + CountLeaves(m_nodes[nodeId].child2);
   //b2Assert(Result == node->leafCount);
end;

function Tb2DynamicTree.ComputeHeight: Int32;
begin
   Result := ComputeHeight(m_root);
end;

function Tb2DynamicTree.CreateProxy(const _aabb: Tb2AABB;
   _userData: Pointer): Int32;
const
   _r: TVector2 = (X: b2_aabbExtension; Y: b2_aabbExtension);
var
   proxyId, iterationCount, tryCount, height: Int32;
begin
   proxyId := AllocateNode;

   // Fatten the aabb.
   with m_nodes[proxyId], aabb do
   begin
      {$IFDEF OP_OVERLOAD}
      lowerBound := _aabb.lowerBound - _r;
      upperBound := _aabb.upperBound + _r;
      {$ELSE}
      lowerBound := Subtract(_aabb.lowerBound, _r);
      upperBound := Add(_aabb.upperBound, _r);
      {$ENDIF}
      userData := _userData;
      leafCount := 1;
   end;

   InsertLeaf(proxyId);
   Result := proxyId;
end;

procedure Tb2DynamicTree.DestroyProxy(proxyId: Int32);
begin
   //b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   //b2Assert(m_nodes[proxyId].IsLeaf());

   RemoveLeaf(proxyId);
   FreeNode(proxyId);
end;

function Tb2DynamicTree.MoveProxy(proxyId: Int32; const aabb: Tb2AABB;
   const displacement: TVector2): Boolean;
const
   _r: TVector2 = (X: b2_aabbExtension; Y: b2_aabbExtension);
var
   b: Tb2AABB;
   d: TVector2;
begin
   //b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   //b2Assert(m_nodes[proxyId].IsLeaf());

   if {$IFDEF OP_OVERLOAD}m_nodes[proxyId].aabb.Contains(aabb)
      {$ELSE}Contains(m_nodes[proxyId].aabb, aabb){$ENDIF} then
   begin
      Result := False;
      Exit;
   end;

   RemoveLeaf(proxyId);

   // Extend AABB.
   b := aabb;
   {$IFDEF OP_OVERLOAD}
   b.lowerBound.SubtractBy(_r);
   b.upperBound.AddBy(_r);
   {$ELSE}
   SubtractBy(b.lowerBound, _r);
   AddBy(b.upperBound, _r);
   {$ENDIF}

   // Predict AABB displacement.
   {$IFDEF OP_OVERLOAD}
   d := b2_aabbMultiplier * displacement;
   {$ELSE}
   d := Multiply(displacement, b2_aabbMultiplier);
   {$ENDIF}

   with b do
   begin
      if d.x < 0.0 then
         lowerBound.x := lowerBound.x + d.x
      else
         upperBound.x := upperBound.x + d.x;

      if d.y < 0.0 then
         lowerBound.y := lowerBound.y + d.y
      else
         upperBound.y := upperBound.y + d.y;
   end;

   m_nodes[proxyId].aabb := b;

   InsertLeaf(proxyId);
   Result := True;
end;

procedure Tb2DynamicTree.Rebalance(iterations: Int32);
var
   i: Integer;
   node, selector: Int32;
   children: PInt32;
   bit: UInt32;
begin
   if m_root = b2_nullNode then
      Exit;

   // Rebalance the tree by removing and re-inserting leaves.
   for i := 0 to iterations - 1 do
   begin
      node := m_root;

      bit := 0;
      while not {$IFDEF OP_OVERLOAD}m_nodes[node].IsLeaf{$ELSE}IsLeaf(m_nodes[node]){$ENDIF} do
      begin
         children := @m_nodes[node].child1;
         selector := (m_path shr bit) and 1;// Child selector based on a bit in the path
         node := PInt32(Integer(children) + SizeOf(Int32) * selector)^; // Select the child nod

         // Keep bit between 0 and 31 because m_path has 32 bits
         // bit = (bit + 1) % 31
         bit := (bit + 1) and $1F;
      end;
      Inc(m_path);

      RemoveLeaf(node);
      InsertLeaf(node);
   end;
end;

function Tb2DynamicTree.GetUserData(proxyId: Int32): Pointer;
begin
   Result := m_nodes[proxyId].userData;
end;

function Tb2DynamicTree.GetFatAABB(proxyId: Int32): Pb2AABB;
begin
   Result := @m_nodes[proxyId].aabb;
end;

procedure Tb2DynamicTree.Query(callback: Tb2GenericCallBackWrapper; const _aabb: Tb2AABB);
var
   nodeId: Int32;
begin
   _GrowableStack.Reset;
   _GrowableStack.Push(m_root);

   while _GrowableStack.GetCount > 0 do
   begin
      nodeId := _GrowableStack.Pop;
      if nodeId = b2_nullNode then
         Continue;

      with m_nodes[nodeId] do
         if b2TestOverlap(aabb, _aabb) then
         begin
            {$IFDEF OP_OVERLOAD}
            if IsLeaf then
            {$ELSE}
            if IsLeaf(m_nodes[nodeId]) then
            {$ENDIF}
            begin
               if not callback.QueryCallback(nodeId) then
                  Exit;
            end
            else
            begin
				       _GrowableStack.Push(child1);
				       _GrowableStack.Push(child2);
            end;
         end;
   end;
end;

procedure Tb2DynamicTree.RayCast(callback: Tb2GenericCallBackWrapper;
   const input: Tb2RayCastInput);
var
   maxFraction, value: Float;
   r, t, v, abs_v, c, h: TVector2;
   segmentAABB: Tb2AABB;
   nodeId: Int32;
   subInput: Tb2RayCastInput;
begin
   {$IFDEF OP_OVERLOAD}
   r := input.p2 - input.p1;
   //b2Assert(r.LengthSquared() > 0.0f);
   r.Normalize;
   {$ELSE}
   r := Subtract(input.p2, input.p1);
   //b2Assert(r.LengthSquared() > 0.0f);
   Normalize(r);
   {$ENDIF}

   // v is perpendicular to the segment.
   v := b2Cross(1.0, r);
   abs_v := b2Abs(v);

   // Separating axis for segment (Gino, p80).
   // |dot(v, p1 - c)| > dot(|v|, h)

   maxFraction := input.maxFraction;

   // Build a bounding box for the segment.
   {$IFDEF OP_OVERLOAD}
   t := input.p1 + maxFraction * (input.p2 - input.p1);
   {$ELSE}
   t := Add(input.p1, Multiply(Subtract(input.p2, input.p1), maxFraction));
   {$ENDIF}
   segmentAABB.lowerBound := b2Min(input.p1, t);
   segmentAABB.upperBound := b2Max(input.p1, t);

   _GrowableStack.Reset;
   _GrowableStack.Push(m_root);

   while _GrowableStack.GetCount > 0 do
   begin
      nodeId := _GrowableStack.Pop;
      if nodeId = b2_nullNode then
         Continue;

      with m_nodes[nodeId] do
      begin
         if not b2TestOverlap(aabb, segmentAABB) then
            Continue;

         // Separating axis for segment (Gino, p80).
         // |dot(v, p1 - c)| > dot(|v|, h)
         {$IFDEF OP_OVERLOAD}
         c := aabb.GetCenter;
         h := aabb.GetExtents;
         if Abs(b2Dot(v, input.p1 - c)) - b2Dot(abs_v, h) > 0.0 then
            Continue;
         {$ELSE}
         c := GetCenter(aabb);
         h := GetExtents(aabb);
         if Abs(b2Dot(v, Subtract(input.p1, c))) - b2Dot(abs_v, h) > 0.0 then
            Continue;
         {$ENDIF}

         {$IFDEF OP_OVERLOAD}
         if IsLeaf then
         {$ELSE}
         if IsLeaf(m_nodes[nodeId]) then
         {$ENDIF}
         begin
            subInput.p1 := input.p1;
            subInput.p2 := input.p2;
            subInput.maxFraction := maxFraction;

            value := callback.RayCastCallback(subInput, nodeId);

            if value = 0.0 then
               Exit; // The client has terminated the ray cast.

            if value > 0.0 then
            begin
               maxFraction := value;
               // Update segment bounding box.
               {$IFDEF OP_OVERLOAD}
               t := input.p1 + maxFraction * (input.p2 - input.p1);
               {$ELSE}
               t := Add(input.p1, Multiply(Subtract(input.p2, input.p1), maxFraction));
               {$ENDIF}
               segmentAABB.lowerBound := b2Min(input.p1, t);
               segmentAABB.upperBound := b2Max(input.p1, t);
            end;
         end
         else
         begin
 			      _GrowableStack.Push(child1);
			      _GrowableStack.Push(child2);
         end;
      end;
   end;
end;

procedure Tb2DynamicTree.Validate;
begin
   CountLeaves(m_root);
end;

{ Tb2BroadPhase }

/// This is used to sort pairs.
function b2PairLessThan(const pair1, pair2: Tb2Pair): Int32; {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
begin
   Result := pair1.proxyIdA - pair2.proxyIdA;
   if Result = 0 then
      Result := pair1.proxyIdB - pair2.proxyIdB;
end;

constructor Tb2BroadPhase.Create;
begin
   m_tree := Tb2DynamicTree.Create;

   m_proxyCount := 0;
   m_pairCapacity := 16;
   m_pairCount := 0;
   SetLength(m_pairBuffer, m_pairCapacity);

   m_moveCapacity := 16;
   m_moveCount := 0;
   SetLength(m_moveBuffer, m_moveCapacity);
end;

destructor Tb2BroadPhase.Destroy;
begin
   m_tree.Free;
end;

procedure Tb2BroadPhase.QuickSortPairBuffer(L, R: Int32);
var
  I, J: Integer;
  P: Pb2Pair;
  T: Tb2Pair;
begin
   repeat
      I := L;
      J := R;
      P := @m_pairBuffer[(L + R) shr 1];
      repeat
         while b2PairLessThan(m_pairBuffer[I], P^) < 0 do
            Inc(I);
         while b2PairLessThan(m_pairBuffer[J], P^) > 0 do
            Dec(J);
         if I <= J then
         begin
            if I <> J then
            begin
               T := m_pairBuffer[I];
               m_pairBuffer[I] := m_pairBuffer[J];
               m_pairBuffer[J] := T;
            end;
            Inc(I);
            Dec(J);
         end;
      until I > J;
      if L < J then
         QuickSortPairBuffer(L, J);
      L := I;
   until I >= R;
end;

procedure Tb2BroadPhase.BufferMove(proxyId: Int32);
begin
   if m_moveCount = m_moveCapacity then
   begin
      m_moveCapacity := m_moveCapacity * 2;
      SetLength(m_moveBuffer, m_moveCapacity);
   end;

   m_moveBuffer[m_moveCount] := proxyId;
   Inc(m_moveCount);
end;

procedure Tb2BroadPhase.UnBufferMove(proxyId: Int32);
var
   i: Integer;
begin
   for i := 0 to m_moveCount - 1 do
      if m_moveBuffer[i] = proxyId then
      begin
         m_moveBuffer[i] := e_nullProxy;
			   Exit;
      end;   
end;

function Tb2BroadPhase.QueryCallback(proxyId: Int32): Boolean;
begin
   // A proxy cannot form a pair with itself.
   if proxyId = m_queryProxyId then
   begin
      Result := True;
      Exit;
   end;

   // Grow the pair buffer as needed.
   if m_pairCount = m_pairCapacity then
   begin
      m_pairCapacity := m_pairCapacity * 2;
      SetLength(m_pairBuffer, m_pairCapacity);
   end;

   m_pairBuffer[m_pairCount].proxyIdA := b2Min(proxyId, m_queryProxyId);
   m_pairBuffer[m_pairCount].proxyIdB := b2Max(proxyId, m_queryProxyId);
   Inc(m_pairCount);

   Result := True;
end;

function Tb2BroadPhase.CreateProxy(const aabb: Tb2AABB; userData: Pointer): Int32;
begin
   Result := m_tree.CreateProxy(aabb, userData);
   Inc(m_proxyCount);
   BufferMove(Result);
end;

procedure Tb2BroadPhase.DestroyProxy(proxyId: Int32);
begin
   UnBufferMove(proxyId);
   Dec(m_proxyCount);
   m_tree.DestroyProxy(proxyId);
end;

procedure Tb2BroadPhase.MoveProxy(proxyId: Int32; const aabb: Tb2AABB;
   const displacement: TVector2);
begin
   if m_tree.MoveProxy(proxyId, aabb, displacement) then
      BufferMove(proxyId);
end;

function Tb2BroadPhase.GetFatAABB(proxyId: Int32): Pb2AABB;
begin
   Result := m_tree.GetFatAABB(proxyId);
end;

function Tb2BroadPhase.GetUserData(proxyId: Int32): Pointer;
begin
   Result := m_tree.GetUserData(proxyId);
end;

function Tb2BroadPhase.TestOverlap(proxyIdA, proxyIdB: Int32): Boolean;
begin
   Result := b2TestOverlap(m_tree.GetFatAABB(proxyIdA)^, m_tree.GetFatAABB(proxyIdB)^);
end;

procedure Tb2BroadPhase.UpdatePairs(callback: Tb2ContactManager);
var
   i: Integer;
   fatAABB: Pb2AABB;
   primaryPair, pair: Pb2Pair;
begin
   // Reset pair buffer
   m_pairCount := 0;

   // Perform tree queries for all moving proxies.
   for i := 0 to m_moveCount - 1 do
   begin
      m_queryProxyId := m_moveBuffer[i];
      if m_queryProxyId = e_nullProxy then
         Continue;

      // We have to query the tree with the fat AABB so that
      // we don't fail to create a pair that may touch later.
      fatAABB := m_tree.GetFatAABB(m_queryProxyId);

      // Query tree, create pairs and add them pair buffer.
      m_tree.Query(Self, fatAABB^);
   end;

   // Reset move buffer
   m_moveCount := 0;

   // Sort the pair buffer to expose duplicates.
   if m_pairCount >= 2 then
      QuickSortPairBuffer(0, m_pairCount - 1);

   // Send the pairs back to the client.
   i := 0;
   while i < m_pairCount do
   begin
      primaryPair := @m_pairBuffer[i];
      callback.AddPair(m_tree.GetUserData(primaryPair^.proxyIdA),
         m_tree.GetUserData(primaryPair^.proxyIdB));
      Inc(i);

      // Skip any duplicate pairs.
      while i < m_pairCount do
      begin
         pair := @m_pairBuffer[i];
         if (pair^.proxyIdA <> primaryPair^.proxyIdA) or (pair^.proxyIdB <> primaryPair^.proxyIdB) then
            Break;
         Inc(i);
      end;
   end;

   // Try to keep the tree balanced.
   m_tree.Rebalance(4);
end;

procedure Tb2BroadPhase.Query(callback: Tb2GenericCallBackWrapper; const aabb: Tb2AABB);
begin
   m_tree.Query(callback, aabb);
end;

procedure Tb2BroadPhase.RayCast(callback: Tb2GenericCallBackWrapper;
   const input: Tb2RayCastInput);
begin
   m_tree.RayCast(callback, input);
end;

function Tb2BroadPhase.ComputeHeight: Int32;
begin
   Result := m_tree.ComputeHeight;
end;

//////////////////////////////////////////////////////////////
/// Fixture & Shapes

{ Tb2Shape }

constructor Tb2Shape.Create;
begin
   m_type := e_unknownShape;
   m_destroyed := False;
end;

destructor Tb2Shape.Destroy;
begin
   m_destroyed := True;
   if Assigned(m_fixture) then
      if Assigned(m_fixture.m_body) then
         m_fixture.m_body.DestroyFixture(m_fixture);
end;

{ Tb2FixtureDef }

constructor Tb2FixtureDef.Create;
begin
   shape := nil;
   userData := nil;
   friction := 0.2;
   restitution := 0.0;
   density := 0.0;
   filter.categoryBits := 1;
   filter.maskBits := $FFFF;
   filter.groupIndex := 0;
   isSensor := False;
end;

{ Tb2Fixture }

constructor Tb2Fixture.Create(body: Tb2Body; def: Tb2FixtureDef;
   AutoFreeShape: Boolean = True);
var
   i: Integer;
   childCount: Int32;
begin
   m_userData := def.userData;
   m_friction := def.friction;
   m_restitution := def.restitution;

   m_body := body;
   m_next := nil;

   m_filter := def.filter;
   m_isSensor := def.isSensor;
   m_density := def.density;

   m_shape := def.shape.Clone;
   if AutoFreeShape then
      def.shape.Free;
   m_shape.m_fixture := Self;

	 // Reserve proxy space
	 childCount := m_shape.GetChildCount;
   SetLength(m_proxies, childCount);
   for i := 0 to childCount - 1 do
   begin
	  	m_proxies[i].fixture := nil;
	  	m_proxies[i].proxyId := e_nullProxy;
   end;
   m_proxyCount := 0;
end;

destructor Tb2Fixture.Destroy;
var
   childCount: Int32;
begin
   // The proxies must be destroyed before calling this.
	 //b2Assert(m_proxyCount == 0);

   if Assigned(m_body) then
      m_body.DestroyFixture(Self, False);

   // Free the child shape.
   m_shape.m_fixture := nil;
   m_shape.Free;
end;

destructor Tb2Fixture.Destroy2;
begin
   if not m_shape.m_destroyed then
   begin
      m_shape.m_fixture := nil;
      m_shape.Free;
   end;
end;

procedure Tb2Fixture.CreateProxies(broadPhase: Tb2BroadPhase; const xf: Tb2Transform);
var
   i: Integer;
begin
   //b2Assert(m_proxyCount == 0);

   // Create proxies in the broad-phase.
   m_proxyCount := m_shape.GetChildCount;

   // Create proxies in the broad-phase.
   for i := 0 to m_proxyCount - 1 do
      with m_proxies[i] do
      begin
         m_shape.ComputeAABB(aabb, xf, i);
         proxyId := broadPhase.CreateProxy(aabb, @m_proxies[i]);
         fixture := Self;
         childIndex := i;
      end;
end;

procedure Tb2Fixture.DestroyProxies(broadPhase: Tb2BroadPhase);
var
   i: Integer;
begin
   // Destroy proxies in the broad-phase.
   for i := 0 to m_proxyCount - 1 do
      with m_proxies[i] do
      begin
         broadPhase.DestroyProxy(proxyId);
         proxyId := e_nullProxy;
      end;

   m_proxyCount := 0;
end;

procedure Tb2Fixture.Synchronize(broadPhase: Tb2BroadPhase; const xf1, xf2: Tb2Transform);
var
   i: Integer;
   aabb1, aabb2: Tb2AABB;
   displacement: TVector2;
begin
   if m_proxyCount = 0 then
      Exit;

   for i := 0 to m_proxyCount - 1 do
      with m_proxies[i] do
      begin
         // Compute an AABB that covers the swept shape (may miss some rotation effect).
         m_shape.ComputeAABB(aabb1, xf1, childIndex);
         m_shape.ComputeAABB(aabb2, xf2, childIndex);

         {$IFDEF OP_OVERLOAD}
         aabb.Combine(aabb1, aabb2);
         displacement := xf2.position - xf1.position;
         {$ELSE}
         Combine(aabb, aabb1, aabb2);
         displacement := Subtract(xf2.position, xf1.position);
         {$ENDIF}
         broadPhase.MoveProxy(proxyId, aabb, displacement);
      end;
end;

function Tb2Fixture.GetType: Tb2ShapeType;
begin
   Result := m_shape.m_type;
end;

procedure Tb2Fixture.SetFilterData(const filter: Tb2Filter);
var
   edge: Pb2ContactEdge;
begin
   m_filter := filter;

   if not Assigned(m_body) then
      Exit;

   // Flag associated contacts for filtering.
   edge := m_body.GetContactList;
   while Assigned(edge) do
      with edge^, contact^ do
      begin
         if (m_fixtureA = Self) or (m_fixtureB = Self) then
            {$IFDEF OP_OVERLOAD}
            FlagForFiltering;
            {$ELSE}
            FlagForFiltering(contact^);
            {$ENDIF}
         edge := next;
      end;
end;

function Tb2Fixture.GetFilterData: Pb2Filter;
begin
   Result := @m_filter;
end;

function Tb2Fixture.TestPoint(const p: TVector2): Boolean;
begin
   Result := m_shape.TestPoint(m_body.m_xf, p);
end;

function Tb2Fixture.RayCast(var output: Tb2RayCastOutput;
   const input: Tb2RayCastInput; childIndex: Int32): Boolean;
begin
   Result := m_shape.RayCast(output, input, m_body.m_xf, childIndex);
end;

procedure Tb2Fixture.GetMassData(var massData: Tb2MassData);
begin
   m_shape.ComputeMass(massData, m_density);
end;

function Tb2Fixture.GetAABB(childIndex: Int32): Pb2AABB;
begin
   //b2Assert(0 <= childIndex && childIndex < m_proxyCount);
	 Result := @m_proxies[childIndex].aabb;
end;

//////////////////////////////////////////////////////////////
// Joints    

{ Tb2Jacobian }

{$IFDEF OP_OVERLOAD}
procedure Tb2Jacobian.SetZero;
begin
   linearA := b2Vec2_Zero;
   linearB := b2Vec2_Zero;
   angularA := 0.0;
   angularB := 0.0;
end;
      
procedure Tb2Jacobian.SetValue(const x1, x2: TVector2; a1, a2: Float);
begin
	 linearA := x1;
   linearB := x2;
   angularA := a1;
   angularB := a2;
end;
      
function Tb2Jacobian.Compute(const x1, x2: TVector2; a1, a2: Float): Float;
begin
   Result := b2Dot(linearA, x1) + angularA * a1 + b2Dot(linearB, x2) + angularB * a2;
end;  
{$ENDIF}

{ Tb2JointDef }

constructor Tb2JointDef.Create;
begin
		JointType := e_unknownJoint;
		userData := nil;
		bodyA := nil;
		bodyB := nil;
		collideConnected := False;
end;

{ Tb2Joint }

constructor Tb2Joint.Create(def: Tb2JointDef);
begin
   //b2Assert(def->bodyA != def->bodyB);
   m_type := def.JointType;
   m_prev := nil;
   m_next := nil;
   m_bodyA := def.bodyA;
   m_bodyB := def.bodyB;
   m_collideConnected := def.collideConnected;
   m_islandFlag := False;
   m_userData := def.userData;

   m_edgeA.joint := nil;
   m_edgeA.other := nil;
   m_edgeA.prev := nil;
   m_edgeA.next := nil;
   m_edgeB.joint := nil;
   m_edgeB.other := nil;
   m_edgeB.prev := nil;
   m_edgeB.next := nil;
end;

function Tb2Joint.IsActive: Boolean;
begin
   Result := m_bodyA.IsActive and m_bodyB.IsActive;
end;

{$IFDEF CONTROLLERS}
//////////////////////////////////////////////////////////////
// Controllers

{ Tb2Controller }

destructor Tb2Controller.Destroy;
begin
   //Remove attached bodies
   Clear;
   inherited;
end;

procedure Tb2Controller.Draw(debugDraw: Tb2DebugDraw);
begin
end;

procedure Tb2Controller.Clear;
var
   edge: Pb2ControllerEdge;
begin
   while Assigned(m_bodyList) do
   begin
      edge := m_bodyList;

      //Remove edge from controller list
      m_bodyList := edge^.nextBody;

      //Remove edge from body list
      if Assigned(edge^.prevController) then
         edge^.prevController^.nextController := edge^.nextController;
      if Assigned(edge^.nextController) then
         edge^.nextController^.prevController := edge^.prevController;
      if edge = edge^.body.m_controllerList then
         edge^.body.m_controllerList := edge^.nextController;

      Dispose(edge)//Free the edge
   end;
   m_bodyCount := 0;
end;

procedure Tb2Controller.AddBody(body: Tb2Body);
var
   edge: Pb2ControllerEdge;
begin
   New(edge);
   edge^.body := body;
   edge^.controller := Self;

   //Add edge to controller list
   edge^.nextBody := m_bodyList;
   edge^.prevBody := nil;
   if Assigned(m_bodyList) then
      m_bodyList^.prevBody := edge;
   m_bodyList := edge;
   Inc(m_bodyCount);

   //Add edge to body list
   edge^.nextController := body.m_controllerList;
   edge^.prevController := nil;
   if Assigned(body.m_controllerList) then
      body.m_controllerList^.prevController := edge;
   body.m_controllerList := edge;
end;

procedure Tb2Controller.RemoveBody(body: Tb2Body);
var
   edge: Pb2ControllerEdge;
begin
   //Assert that the controller is not empty
   //b2Assert(m_bodyCount>0);

   //Find the corresponding edge
   edge := m_bodyList;
   while Assigned(edge) and (edge^.body <> body) do
      edge := edge^.nextBody;

   //Assert that we are removing a body that is currently attached to the controller
   //b2Assert(edge!=NULL);

   //Remove edge from controller list
   if Assigned(edge^.prevBody) then
      edge^.prevBody^.nextBody := edge^.nextBody;
   if Assigned(edge^.nextBody) then
      edge^.nextBody^.prevBody := edge^.prevBody;
   if edge = m_bodyList then
      m_bodyList := edge^.nextBody;
   Dec(m_bodyCount);

   //Remove edge from body list
   if Assigned(edge^.prevController) then
      edge^.prevController^.nextController := edge^.nextController;
   if Assigned(edge^.nextController) then
      edge^.nextController^.prevController := edge^.prevController;
   if edge = body.m_controllerList then
      body.m_controllerList := edge^.nextController;

   Dispose(edge); //Free the edge
end;
{$ENDIF}

//////////////////////////////////////////////////////////////
// Body

{ Tb2BodyDef }

constructor Tb2BodyDef.Create;
begin
   userData := nil;
   ignoreColliding := False;
   position := b2Vec2_Zero;
   angle := 0.0;
   linearVelocity := b2Vec2_Zero;
   angularVelocity := 0.0;
   linearDamping := 0.0;
   angularDamping := 0.0;
   allowSleep := True;
   awake := True;
   fixedRotation := False;
   bullet := False;
   bodyType := b2_staticBody;
   active := True;
   inertiaScale := 1.0;
end;

{ Tb2Body }

constructor Tb2Body.Create(bd: Tb2BodyDef; world: Tb2World);
begin
   //b2Assert(bd->position.IsValid());
   //b2Assert(bd->linearVelocity.IsValid());
   //b2Assert(b2IsValid(bd->angle));
   //b2Assert(b2IsValid(bd->angularVelocity));
   //b2Assert(b2IsValid(bd->inertiaScale) && bd->inertiaScale >= 0.0f);
   //b2Assert(b2IsValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
   //b2Assert(b2IsValid(bd->linearDamping) && bd->linearDamping >= 0.0f);

   m_flags := 0;

   if bd.bullet then
      m_flags := m_flags or e_body_bulletFlag;
   if bd.fixedRotation then
      m_flags := m_flags or e_body_fixedRotationFlag;
   if bd.allowSleep then
      m_flags := m_flags or e_body_autoSleepFlag;
   if bd.awake then
      m_flags := m_flags or e_body_awakeFlag;
   if bd.active then
      m_flags := m_flags or e_body_activeFlag;
   if bd.ignoreColliding then
      m_flags := m_flags or e_body_ignoreCollideFlag;

   m_world := world;

   m_xf.position := bd.position;
   {$IFDEF OP_OVERLOAD}
   m_xf.R.SetValue(bd.angle);
   {$ELSE}
   SetValue(m_xf.R, bd.angle);
   {$ENDIF}

   m_sweep.localCenter := b2Vec2_Zero;
   m_sweep.a := bd.angle;
   m_sweep.a0 := bd.angle;
   m_sweep.c0 := b2Mul(m_xf, m_sweep.localCenter);
   m_sweep.c := m_sweep.c0;

   m_jointList := nil;
   m_contactList := nil;
   m_prev := nil;
   m_next := nil;

	 m_linearVelocity := bd.linearVelocity;
   m_angularVelocity := bd.angularVelocity;
   m_linearDamping := bd.linearDamping;
   m_angularDamping := bd.angularDamping;

   m_force := b2Vec2_Zero;
   m_torque := 0.0;
   m_sleepTime := 0.0;

   m_type := bd.bodyType;
   if m_type = b2_dynamicBody then
   begin
      m_mass := 1.0;
      m_invMass := 1.0;
   end
   else
   begin
      m_mass := 0.0;
      m_invMass := 0.0;
   end;

   m_I := 0.0;
   m_invI := 0.0;

   m_userData := bd.userData;

   m_fixtureList := nil;
   m_fixtureCount := 0;

   {$IFDEF CONTROLLERS}
   m_controllerList := nil;
   m_controllerCount := 0;
   {$ENDIF}

   ComputeStoredInertia;
end;

destructor Tb2Body.Destroy;
begin
   if Assigned(m_world) then
      m_world.DestroyBody(Self, False);
end;

destructor Tb2Body.Destroy2;
begin
end;

procedure Tb2Body.ComputeStoredInertia;
begin
   m_storedInertia := m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
end;

function Tb2Body.CreateFixture(def: Tb2FixtureDef;
   AutoFreeFixtureDef: Boolean = True; AutoFreeShape: Boolean = True;
   AutoResetMassData: Boolean = True): Tb2Fixture;
begin
   Result := nil;
   //b2Assert(m_world->IsLocked() == false);
   if m_world.IsLocked then
   begin
      if AutoFreeFixtureDef then
         def.Free;
      Exit;
   end;

   Result := Tb2Fixture.Create(Self, def, AutoFreeShape);
   if m_flags and e_body_activeFlag <> 0 then
      Result.CreateProxies(m_world.m_contactManager.m_broadPhase, m_xf);

   Result.m_body := Self;
   Result.m_next := m_fixtureList;
   m_fixtureList := Result;
   Inc(m_fixtureCount);

   // Adjust mass properties if needed.
   if (Result.m_density > 0.0) and AutoResetMassData then
      ResetMassData;

   // Let the world know we have a new fixture. This will cause new contacts
   // to be created at the beginning of the next time step.
   m_world.m_flags := m_world.m_flags or e_world_newFixture;

   if AutoFreeFixtureDef then
      def.Free;
end;

function Tb2Body.CreateFixture(shape: Tb2Shape; density: Float;
   AutoFreeShape: Boolean = True; AutoResetMassData: Boolean = True): Tb2Fixture;
var
   def: Tb2FixtureDef;
begin
	 def := Tb2FixtureDef.Create;
	 def.shape := shape;
	 def.density := density;
	 Result := CreateFixture(def, True, AutoFreeShape, AutoResetMassData);
end;

procedure Tb2Body.DestroyFixture(fixture: Tb2Fixture; DoFree: Boolean = True);
var
   node: Tb2Fixture;
   found: Boolean;
   edge: Pb2ContactEdge;
   c: Pb2Contact;
begin
   //b2Assert(m_world->IsLocked() == false);
   if m_world.IsLocked then
      Exit;

   //b2Assert(fixture->m_body == this);

   // Remove the fixture from this body's singly linked list.
   //b2Assert(m_fixtureCount > 0);
   node := m_fixtureList;
   found := False;
   while Assigned(node) do
   begin
      if node = fixture then
      begin
         found := True;
         Break;
      end;
      node := node.m_next;
   end;

   // You tried to remove a shape that is not attached to this body.
   if not found then
      Exit;

   // Destroy any contacts associated with the fixture.
   edge := m_contactList;
   while Assigned(edge) do
   begin
      c := edge^.contact;
      edge := edge^.next;

      if (fixture = c^.m_fixtureA) or (fixture = c^.m_fixtureB) then
      begin
         // This destroys the contact and removes it from
         // this body's contact list.
         m_world.m_contactManager.Destroy(c);
      end;
   end;

   if m_flags and e_body_activeFlag <> 0 then
   begin
      //b2Assert(fixture->m_proxyId != b2BroadPhase::e_nullProxy);
      fixture.DestroyProxies(m_world.m_contactManager.m_broadPhase);
   end
   else
      ;//b2Assert(fixture->m_proxyId == b2BroadPhase::e_nullProxy);

   if DoFree then
      fixture.Destroy2; // Call a destructor without side effects.
   Dec(m_fixtureCount);
   if m_fixtureCount = 0 then
      m_fixtureList := nil;
   // Reset the mass data.
   ResetMassData;
end;

procedure Tb2Body.DestroyFixtures(ResetMass: Boolean);
var
   node, tmpnode: Tb2Fixture;
   edge: Pb2ContactEdge;
   c: Pb2Contact;
begin
   if m_world.IsLocked then
      Exit;

   // Destroy all contacts
   edge := m_contactList;
   while Assigned(edge) do
   begin
      c := edge^.contact;
      m_world.m_contactManager.Destroy(c);
      edge := edge^.next;
   end;

   if m_flags and e_body_activeFlag <> 0 then
   begin
      node := m_fixtureList;
      while Assigned(node) do
      begin
         node.DestroyProxies(m_world.m_contactManager.m_broadPhase);
         node := node.m_next;
      end;
   end;

   // Free all fixtures
   node := m_fixtureList;
   while Assigned(node) do
   begin
      tmpnode := node.m_next;
      node.Destroy2;
      node := tmpnode;
   end;

   m_fixtureList := nil;
   m_fixtureCount := 0;

   if ResetMass then
      ResetMassData;
   // If not ResetMass, this body becomes a shape-less massed object
end;

procedure Tb2Body.SynchronizeFixtures;
var
   xf1: Tb2Transform;
   f: Tb2Fixture;
begin
   {$IFDEF OP_OVERLOAD}
   xf1.R.SetValue(m_sweep.a0);
   xf1.position := m_sweep.c0 - b2Mul(xf1.R, m_sweep.localCenter);
   {$ELSE}
   SetValue(xf1.R, m_sweep.a0);
   xf1.position := Subtract(m_sweep.c0, b2Mul(xf1.R, m_sweep.localCenter));
   {$ENDIF}

   f := m_fixtureList;
   while Assigned(f) do
   begin
      f.Synchronize(m_world.m_contactManager.m_broadPhase, xf1, m_xf);
      f := f.m_next;
   end;
end;

procedure Tb2Body.SynchronizeTransform;
begin
   {$IFDEF OP_OVERLOAD}
   m_xf.R.SetValue(m_sweep.a);
   m_xf.position := m_sweep.c - b2Mul(m_xf.R, m_sweep.localCenter);
   {$ELSE}
   SetValue(m_xf.R, m_sweep.a);
   m_xf.position := Subtract(m_sweep.c, b2Mul(m_xf.R, m_sweep.localCenter));
   {$ENDIF}
end;

function Tb2Body.ShouldCollide(other: Tb2Body): Boolean;
var
   jn: Pb2JointEdge;
begin
   if IsCollidingIgnored then
   begin
      Result := False;
      Exit;
   end;

   // At least one body should be dynamic.
   if (m_type <> b2_dynamicBody) and (other.m_type <> b2_dynamicBody) then
   begin
      Result := False;
      Exit;
   end;

   // Does a joint prevent collision?
   jn := m_jointList;
   while Assigned(jn) do
   begin
      if jn^.other = other then
         if not jn^.joint.m_collideConnected then
         begin
            Result := False;
            Exit;
         end;
      jn := jn^.next;
   end;
   Result := True;
end;

procedure Tb2Body.Advance(alpha: Float);
begin
   // Advance to the new safe time.
   {$IFDEF OP_OVERLOAD}
   m_sweep.Advance(alpha);
   {$ELSE}
   UPhysics2DTypes.Advance(m_sweep, alpha);
   {$ENDIF}
   m_sweep.c := m_sweep.c0;
   m_sweep.a := m_sweep.a0;
   SynchronizeTransform;
end;

procedure Tb2Body.SetTransform(const position: TVector2; angle: Float);
var
   f: Tb2Fixture;
begin
   //b2Assert(m_world->m_lock == False);
   if m_world.IsLocked then
      Exit;

   {$IFDEF OP_OVERLOAD}
   m_xf.R.SetValue(angle);
   {$ELSE}
   SetValue(m_xf.R, angle);
   {$ENDIF}
   m_xf.position := position;

   m_sweep.c0 := b2Mul(m_xf, m_sweep.localCenter);
   m_sweep.c := m_sweep.c0;
   m_sweep.a0 := angle;
   m_sweep.a := angle;

   f := m_fixtureList;
   while Assigned(f) do
   begin
      f.Synchronize(m_world.m_contactManager.m_broadPhase, m_xf, m_xf);
      f := f.m_next;
   end;
   m_world.m_contactManager.FindNewContacts;
end;

function Tb2Body.GetTransform: Pb2Transform;
begin
   Result := @m_xf;
end;

procedure Tb2Body.SetLinearVelocity(const v: TVector2);
begin
   if m_type = b2_staticBody then
      Exit;

   if b2Dot(v, v) > 0.0 then
      SetAwake(True);

   m_linearVelocity := v;
end;

procedure Tb2Body.SetAngularVelocity(omega: Float);
begin
   if m_type = b2_staticBody then
      Exit;

   if omega * omega > 0.0 then
      SetAwake(True);

   m_angularVelocity := omega;
end;

procedure Tb2Body.ApplyForce(const force, point: TVector2);
begin
   if m_type <> b2_dynamicBody then
      Exit;
   if not IsAwake then
      SetAwake(True);
   {$IFDEF OP_OVERLOAD}
	 m_force.AddBy(force);
   m_torque := m_torque + b2Cross(point - m_sweep.c, force);
   {$ELSE}
   AddBy(m_force, force);
   m_torque := m_torque + b2Cross(Subtract(point, m_sweep.c), force);
   {$ENDIF}
end;

procedure Tb2Body.ApplyTorque(torque: Float);
begin
   if m_type <> b2_dynamicBody then
      Exit;
   if not IsAwake then
      SetAwake(True);
	 m_torque := m_torque + torque;
end;

procedure Tb2Body.ApplyLinearImpulse(const impulse, point: TVector2);
begin
   if m_type <> b2_dynamicBody then
      Exit;
   if not IsAwake then
      SetAwake(True);
   {$IFDEF OP_OVERLOAD}
   m_linearVelocity.AddBy(m_invMass * impulse);
   m_angularVelocity := m_angularVelocity + m_invI * b2Cross(point -
      m_sweep.c, impulse);
   {$ELSE}
   AddBy(m_linearVelocity, Multiply(impulse, m_invMass));
   m_angularVelocity := m_angularVelocity + m_invI * b2Cross(Subtract(point,
      m_sweep.c), impulse);
   {$ENDIF}
end;

procedure Tb2Body.ApplyAngularImpulse(impulse: Float);
begin
   if m_type <> b2_dynamicBody then
      Exit;

   if not IsAwake then
      SetAwake(True);
   m_angularVelocity := m_angularVelocity + m_invI * impulse;
end;

procedure Tb2Body.GetMassData(var data: Tb2MassData);
begin
   data.mass := m_mass;
   data.I := m_storedInertia;
   data.center := m_sweep.localCenter;
end;

procedure Tb2Body.SetMassData(const data: Tb2MassData);
var
   oldCenter: TVector2;
begin
   //b2Assert(m_world.IsLocked() == false);
   if m_world.IsLocked then
      Exit;

   if m_type <> b2_dynamicBody then
      Exit;

   m_invMass := 0.0;
   m_I := 0.0;
   m_invI := 0.0;

   m_mass := data.mass;
   if m_mass <= 0.0 then
      m_mass := 1.0;
   m_invMass := 1.0 / m_mass;

   if (data.I > 0.0) and ((m_flags and e_body_fixedRotationFlag) = 0) then
   begin
      m_I := data.I - m_mass * b2Dot(data.center, data.center);
      //b2Assert(m_I > 0.0f);
      m_invI := 1.0 / m_I;
   end;

   // Move center of mass.
   oldCenter := m_sweep.c;
   m_sweep.localCenter := data.center;
   m_sweep.c0 := b2Mul(m_xf, m_sweep.localCenter);
   m_sweep.c := m_sweep.c0;

   // Update center of mass velocity.
   {$IFDEF OP_OVERLOAD}
	 m_linearVelocity.AddBy(b2Cross(m_angularVelocity, m_sweep.c - oldCenter));
   {$ELSE}
   AddBy(m_linearVelocity, b2Cross(m_angularVelocity, Subtract(m_sweep.c, oldCenter)));
   {$ENDIF}

   ComputeStoredInertia;
end;

procedure Tb2Body.ResetMassData;
var
   f: Tb2Fixture;
   center, oldCenter: TVector2;
   massData: Tb2MassData;
begin
   // Compute mass data from shapes. Each shape has its own density.
   m_mass := 0.0;
   m_invMass := 0.0;
   m_I := 0.0;
   m_invI := 0.0;
   m_sweep.localCenter := b2Vec2_Zero;

   // Static and kinematic bodies have zero mass.
   if (m_type = b2_staticBody) or (m_type = b2_kinematicBody) then
   begin
      m_sweep.c0 := m_xf.position;
      m_sweep.c := m_xf.position;
      Exit;
   end;

   //b2Assert(m_type == b2_dynamicBody);

   // Accumulate mass over all fixtures.
   center := b2Vec2_zero;

   f := m_fixtureList;
   while Assigned(f) do
   begin
      if IsZero(f.m_density) then
      begin
         f := f.m_next;
         Continue;
      end;

      f.GetMassData(massData);
      m_mass := m_mass + massData.mass;
      {$IFDEF OP_OVERLOAD}
      center.AddBy(massData.mass * massData.center);
      {$ELSE}
      AddBy(center, Multiply(massData.center, massData.mass));
      {$ENDIF}
      m_I := m_I + massData.I;
      f := f.m_next;
   end;

   // Compute center of mass.
   if m_mass > 0.0 then
   begin
      m_invMass := 1.0 / m_mass;
      {$IFDEF OP_OVERLOAD}
      center.MultiplyBy(m_invMass);
      {$ELSE}
      MultiplyBy(center, m_invMass);
      {$ENDIF}
   end
   else
   begin
		  // Force all dynamic bodies to have a positive mass.
		  m_mass := 1.0;
		  m_invMass := 1.0;
   end;

   if (m_I > 0.0) and ((m_flags and e_body_fixedRotationFlag) = 0) then
   begin
      // Center the inertia about the center of mass.
      m_I := m_I - m_mass * b2Dot(center, center);
      //b2Assert(m_I > 0.0f);
      m_invI := 1.0 / m_I;
   end
   else
   begin
      m_I := 0.0;
      m_invI := 0.0;
   end;

   // Move center of mass.
   oldCenter := m_sweep.c;
   m_sweep.localCenter := center;
   m_sweep.c := b2Mul(m_xf, m_sweep.localCenter);
   m_sweep.c0 := m_sweep.c;

	 // Update center of mass velocity.
   {$IFDEF OP_OVERLOAD}
	 m_linearVelocity.AddBy(b2Cross(m_angularVelocity, m_sweep.c - oldCenter));
   {$ELSE}
   AddBy(m_linearVelocity, b2Cross(m_angularVelocity, Subtract(m_sweep.c, oldCenter)));
   {$ENDIF}

   ComputeStoredInertia;
end;

procedure Tb2Body.SetType(atype: Tb2BodyType);
var
   ce: Pb2ContactEdge;
begin
   if m_type = atype then
      Exit;

   m_type := atype;
   ResetMassData;

   if m_type = b2_staticBody then
   begin
      m_linearVelocity := b2Vec2_Zero;
      m_angularVelocity := 0.0;
   end;

   SetAwake(True);
   m_force := b2Vec2_Zero;
   m_torque := 0.0;

   // Since the body type changed, we need to flag contacts for filtering.
   ce := m_contactList;
   while Assigned(ce) do
   begin
      {$IFDEF OP_OVERLOAD}
      ce^.contact^.FlagForFiltering;
      {$ELSE}
      FlagForFiltering(ce^.contact^);
      {$ENDIF}
      ce := ce^.next;
   end;
end;

function Tb2Body.GetWorldPoint(const localPoint: TVector2): TVector2;
begin
   Result := b2Mul(m_xf, localPoint);
end;

function Tb2Body.GetWorldVector(const localVector: TVector2): TVector2;
begin
   Result := b2Mul(m_xf.R, localVector);
end;

function Tb2Body.GetLocalPoint(const worldPoint: TVector2): TVector2;
begin
   Result := b2MulT(m_xf, worldPoint);
end;

function Tb2Body.GetLocalVector(const worldVector: TVector2): TVector2;
begin
   Result := b2MulT(m_xf.R, worldVector);
end;

function Tb2Body.GetLinearVelocityFromWorldPoint(
   const worldPoint: TVector2): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
   {$ELSE}
   Result := Add(m_linearVelocity, b2Cross(m_angularVelocity,
      Subtract(worldPoint, m_sweep.c)));
   {$ENDIF}
end;

function Tb2Body.GetLinearVelocityFromLocalPoint(
   const localPoint: TVector2): TVector2;
begin
   Result := GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
end;

function Tb2Body.IsBullet: Boolean;
begin
   Result := (m_flags and e_body_bulletFlag) = e_body_bulletFlag;
end;

procedure Tb2Body.SetBullet(flag: Boolean);
begin
   if flag then
      m_flags := m_flags or e_body_bulletFlag
   else
      m_flags := m_flags and (not e_body_bulletFlag);
end;

procedure Tb2Body.SetSleepingAllowed(flag: Boolean);
begin
   if flag then
      m_flags := m_flags or e_body_autoSleepFlag
   else
   begin
      m_flags := m_flags and (not e_body_autoSleepFlag);
		  SetAwake(True);
   end;
end;

function Tb2Body.IsSleepingAllowed: Boolean;
begin
   Result := (m_flags and e_body_autoSleepFlag) = e_body_autoSleepFlag;
end;

procedure Tb2Body.SetAwake(flag: Boolean);
begin
   if flag then
   begin
      if m_flags and e_body_awakeFlag = 0 then
      begin
         m_flags := m_flags or e_body_awakeFlag;
         m_sleepTime := 0.0;
      end;
   end
   else
   begin
      m_flags := m_flags and (not e_body_awakeFlag);
      m_sleepTime := 0.0;
      m_linearVelocity := b2Vec2_Zero;
      m_angularVelocity := 0.0;
      m_force := b2Vec2_Zero;
      m_torque := 0.0;
   end;
end;

function Tb2Body.IsAwake: Boolean;
begin
   Result := (m_flags and e_body_awakeFlag) = e_body_awakeFlag;
end;

procedure Tb2Body.SetActive(flag: Boolean);
var
   f: Tb2Fixture;
   ce, ce0: Pb2ContactEdge;
begin
   if flag = IsActive then
      Exit;

   if flag then
   begin
      m_flags := m_flags or e_body_activeFlag;

      // Create all proxies.
      f := m_fixtureList;
      while Assigned(f) do
      begin
         f.CreateProxies(m_world.m_contactManager.m_broadPhase, m_xf);
         f := f.m_next;
      end;
      // Contacts are created the next time step.
   end
   else
   begin
      m_flags := m_flags and (not e_body_activeFlag);

      // Destroy all proxies.
      f := m_fixtureList;
      while Assigned(f) do
      begin
         f.DestroyProxies(m_world.m_contactManager.m_broadPhase);
         f := f.m_next;
      end;

      // Destroy the attached contacts.
      ce := m_contactList;
      while Assigned(ce) do
      begin
         ce0 := ce;
         ce := ce^.next;
         m_world.m_contactManager.Destroy(ce0^.contact);
      end;
      m_contactList := nil;
   end;
end;

function Tb2Body.IsActive: Boolean;
begin
   Result := (m_flags and e_body_activeFlag) = e_body_activeFlag;
end;

procedure Tb2Body.SetIgnoreColliding(flag: Boolean);
begin
   if flag then
      m_flags := m_flags or e_body_ignoreCollideFlag
   else
      m_flags := m_flags and (not e_body_ignoreCollideFlag);
end;

function Tb2Body.IsCollidingIgnored: Boolean;
begin
   Result := (m_flags and e_body_ignoreCollideFlag) = e_body_ignoreCollideFlag;
end;

procedure Tb2Body.SetFixedRotation(flag: Boolean);
begin
   if flag then
      m_flags := m_flags or e_body_fixedRotationFlag
   else
      m_flags := m_flags and (not e_body_fixedRotationFlag);
   ResetMassData;
end;

function Tb2Body.IsFixedRotation: Boolean;
begin
   Result := (m_flags and e_body_fixedRotationFlag) = e_body_fixedRotationFlag;
end;

///////////////////////////////////////////////
// Specific implementations

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
procedure b2GetPointStates(var state1, state2: Tb2PointStateArray;
   const manifold1, manifold2: Tb2Manifold);
var
   i, j: Integer;
   key: UInt32;
begin
   for i := 0 to High(state1) do
   begin
      state1[i] := b2_nullState;
      state2[i] := b2_nullState;
   end;

   // Detect persists and removes.
   for i := 0 to manifold1.pointCount - 1 do
   begin
      key := manifold1.points[i].id.key; 
      state1[i] := b2_removeState;            
      for j := 0 to manifold2.pointCount - 1 do
         if manifold2.points[j].id.key = key then
         begin
            state1[i] := b2_persistState;
            Break;
         end;
   end;

   // Detect persists and adds.
   for i := 0 to manifold2.pointCount - 1 do
   begin
      key := manifold2.points[i].id.key;  
      state2[i] := b2_addState;  
      for j := 0 to manifold1.pointCount - 1 do
         if manifold1.points[j].id.key = key then
         begin
            state2[i] := b2_persistState;
            Break;
         end;         
   end;
end;

procedure b2CollideCircles(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
var
   pA, pB, d: TVector2;
   circleA, circleB: Tb2CircleShape;
begin
   if ABfixture then
   begin
      circleA := Tb2CircleShape(Tb2Fixture(A).m_shape);
      circleB := Tb2CircleShape(Tb2Fixture(B).m_shape);
   end
   else
   begin
      circleA := Tb2CircleShape(A);
      circleB := Tb2CircleShape(B);
   end;

   manifold.pointCount := 0;

   pA := b2Mul(xfA, circleA.m_p);
   pB := b2Mul(xfB, circleB.m_p);

   {$IFDEF OP_OVERLOAD}
   d := pB - pA;
   {$ELSE}
   d := Subtract(pB, pA);
   {$ENDIF}
   if b2Dot(d, d) > Sqr(circleA.m_radius + circleB.m_radius) then
      Exit;

   with manifold do
   begin
      manifoldType := e_manifold_circles;
      localPoint := circleA.m_p;
      localNormal := b2Vec2_Zero;
      pointCount := 1;

      points[0].localPoint := circleB.m_p;
      points[0].id.key := 0;
   end;
end;

procedure b2CollidePolygonAndCircle(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
var
   circle: Tb2CircleShape;
   polygon: Tb2PolygonShape;
   i: Integer;
   c, cLocal, v1, v2, faceCenter: TVector2;
   normalIndex: Int32;
   _separation, radius: Float;
   s, u1, u2: Float;
begin
   if ABfixture then
   begin
      polygon := Tb2PolygonShape(Tb2Fixture(A).m_shape);
      circle := Tb2CircleShape(Tb2Fixture(B).m_shape);
   end
   else
   begin
      polygon := Tb2PolygonShape(A);
      circle := Tb2CircleShape(B);
   end;

   manifold.pointCount := 0;
   // Compute circle position in the frame of the polygon.
   c := b2Mul(xfB, circle.m_p);
   cLocal := b2MulT(xfA, c);

   // Find the min separating edge.
   normalIndex := 0;
   _separation := -FLT_MAX;
   radius := circle.m_radius + polygon.m_radius;

   for i := 0 to polygon.m_vertexCount - 1 do
   begin
      {$IFDEF OP_OVERLOAD}
      s := b2Dot(polygon.m_normals[i], cLocal - polygon.m_vertices[i]);
      {$ELSE}
      s := b2Dot(polygon.m_normals[i], Subtract(cLocal, polygon.m_vertices[i]));
      {$ENDIF}
      if s > radius then
         Exit;

      if s > _separation then
      begin
         _separation := s;
         normalIndex := i;
      end;
   end;

   // Vertices that subtend the incident face.
   v1 := polygon.m_vertices[normalIndex];
   if normalIndex + 1 < polygon.m_vertexCount then
      v2 := polygon.m_vertices[normalIndex + 1]
   else
      v2 := polygon.m_vertices[0];

   // If the center is inside the polygon ...
   if _separation < FLT_EPSILON then
      with manifold do
      begin
         pointCount := 1;
         manifoldType := e_manifold_faceA;
         localNormal := polygon.m_normals[normalIndex];
         localPoint := b2MiddlePoint(v1, v2);
         points[0].localPoint := circle.m_p;
         points[0].id.key := 0;
         Exit;
      end;

   // Compute barycentric coordinates
   {$IFDEF OP_OVERLOAD}
   u1 := b2Dot(cLocal - v1, v2 - v1);
   u2 := b2Dot(cLocal - v2, v1 - v2);
   {$ELSE}
   u1 := b2Dot(Subtract(cLocal, v1), Subtract(v2, v1));
   u2 := b2Dot(Subtract(cLocal, v2), Subtract(v1, v2));
   {$ENDIF}
   if u1 <= 0.0 then
   begin
      if b2DistanceSquared(cLocal, v1) > radius * radius then
         Exit;

      with manifold do
      begin
         pointCount := 1;
         manifoldType := e_manifold_faceA;
         {$IFDEF OP_OVERLOAD}
         localNormal := cLocal - v1;
         localNormal.Normalize;
         {$ELSE}
         localNormal := Subtract(cLocal, v1);
         Normalize(localNormal);
         {$ENDIF}
         localPoint := v1;
         points[0].localPoint := circle.m_p;
         points[0].id.key := 0;
      end;
   end
   else if u2 <= 0.0 then
   begin
      if b2DistanceSquared(cLocal, v2) > radius * radius then
         Exit;

      with manifold do
      begin
         pointCount := 1;
         manifoldType := e_manifold_faceA;
         {$IFDEF OP_OVERLOAD}
         localNormal := cLocal - v2;
         localNormal.Normalize;
         {$ELSE}
         localNormal := Subtract(cLocal, v2);
         Normalize(localNormal);
         {$ENDIF}
         localPoint := v2;
         points[0].localPoint := circle.m_p;
         points[0].id.key := 0;
      end;
   end
   else
   begin
      faceCenter := b2MiddlePoint(v1, v2);
      {$IFDEF OP_OVERLOAD}
      _separation := b2Dot(cLocal - faceCenter, polygon.m_normals[normalIndex]);
      {$ELSE}
      _separation := b2Dot(Subtract(cLocal, faceCenter), polygon.m_normals[normalIndex]);
      {$ENDIF}
      if _separation > radius then
         Exit;

      with manifold do
      begin
         pointCount := 1;
         manifoldType := e_manifold_faceA;
         localNormal := polygon.m_normals[normalIndex];
         localPoint := faceCenter;
         points[0].localPoint := circle.m_p;
         points[0].id.key := 0;
      end;
   end;
end;

type
   Tb2EdgeType = (b2_isolated, b2_concave, b2_flat, b2_convex);

/// Compute the collision manifold between an edge and a circle.
procedure b2CollideEdgeAndCircle(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
var
   edgeA: Tb2EdgeShape;
   circleB: Tb2CircleShape;
   Q, pA, pB, e, P, d, n: TVector2;
   u, v, radius, dd, den: Float;
   cf: Tb2ContactFeature;
begin
   if ABfixture then
   begin
      edgeA := Tb2EdgeShape(Tb2Fixture(A).m_shape);
      circleB := Tb2CircleShape(Tb2Fixture(B).m_shape);
   end
   else
   begin
      edgeA := Tb2EdgeShape(A);
      circleB := Tb2CircleShape(B);
   end;

   manifold.pointCount := 0;

   // Compute circle in frame of edge
   Q := b2MulT(xfA, b2Mul(xfB, circleB.m_p));

   pA := edgeA.m_vertex1;
   pB := edgeA.m_vertex2;
   {$IFDEF OP_OVERLOAD}
   e := pB - pA;

   // Barycentric coordinates
   u := b2Dot(e, pB - Q);
   v := b2Dot(e, Q - pA);
   {$ELSE}
   e := Subtract(pB, pA);

   // Barycentric coordinates
   u := b2Dot(e, Subtract(pB, Q));
   v := b2Dot(e, Subtract(Q, pA));
   {$ENDIF}

   radius := edgeA.m_radius + circleB.m_radius;

   cf.indexB := 0;
   cf.typeB := e_contact_feature_vertex;

   // Region pA
   if v <= 0.0 then
   begin
      {$IFDEF OP_OVERLOAD}
      d := Q - pA;
      {$ELSE}
      d := Subtract(Q, pA);
      {$ENDIF}
      dd := b2Dot(d, d);
      if dd > radius * radius then
         Exit;

      // Is there an edge connected to pA?
      if edgeA.m_hasVertex0 then
         // Is the circle in Region AB of the previous edge?
         {$IFDEF OP_OVERLOAD}
         if b2Dot(pA - edgeA.m_vertex0, pA - Q) > 0.0 then
         {$ELSE}
         if b2Dot(Subtract(pA, edgeA.m_vertex0), Subtract(pA, Q)) > 0.0 then
         {$ENDIF}
            Exit;

      cf.indexA := 0;
      cf.typeA := e_contact_feature_vertex;
      with manifold do
      begin
         pointCount := 1;
         manifoldType := e_manifold_circles;
         localNormal := b2Vec2_Zero;
         localPoint := pA;
         points[0].id.key := 0;
         points[0].id.cf := cf;
         points[0].localPoint := circleB.m_p;
      end;
      Exit;
   end;

   // Region pB
   if u <= 0.0 then
   begin
      {$IFDEF OP_OVERLOAD}
      d := Q - pB;
      {$ELSE}
      d := Subtract(Q, pB);
      {$ENDIF}
      dd := b2Dot(d, d);
      if dd > radius * radius then
         Exit;

      // Is there an edge connected to pB?
      if edgeA.m_hasVertex3 then
         // Is the circle in Region AB of the next edge?
         {$IFDEF OP_OVERLOAD}
         if b2Dot(edgeA.m_vertex3 - pB, Q - pB) > 0.0 then
         {$ELSE}
         if b2Dot(Subtract(edgeA.m_vertex3, pB), Subtract(Q, pB)) > 0.0 then
         {$ENDIF}
            Exit;

      cf.indexA := 1;
      cf.typeA := e_contact_feature_vertex;
      with manifold do
      begin
         pointCount := 1;
         manifoldType := e_manifold_circles;
         localNormal := b2Vec2_Zero;
         localPoint := pB;
         points[0].id.key := 0;
         points[0].id.cf := cf;
         points[0].localPoint := circleB.m_p;
      end;
      Exit;
   end;

   // Region AB
   den := b2Dot(e, e);
   //b2Assert(den > 0.0f);
   {$IFDEF OP_OVERLOAD}
   P := (1.0 / den) * (u * pA + v * pB);
   d := Q - P;
   {$ELSE}
   P := Multiply(Add(Multiply(pA, u), Multiply(pB, v)), 1.0 / den);
   d := Subtract(Q, P);
   {$ENDIF}
   dd := b2Dot(d, d);
   if dd > radius * radius then
      Exit;

   n.x := -e.y;
   n.y := e.x;
   {$IFDEF OP_OVERLOAD}
   if b2Dot(n, Q - pA) < 0.0 then
   {$ELSE}
   if b2Dot(n, Subtract(Q, pA)) < 0.0 then
   {$ENDIF}
     SetValue(n, -n.x, -n.y);
   {$IFDEF OP_OVERLOAD}
   n.Normalize;
   {$ELSE}
   Normalize(n);
   {$ENDIF}

   cf.indexA := 0;
   cf.typeA := e_contact_feature_face;
   with manifold do
   begin
      pointCount := 1;
      manifoldType := e_manifold_faceA;
      localNormal := n;
      localPoint := pA;
      points[0].id.key := 0;
      points[0].id.cf := cf;
      points[0].localPoint := circleB.m_p;
   end;
end;

const
   e_ep_edgeA = 1;
   e_ep_edgeB = 2;

type
   Tb2EPAxis = record
      AxisType: Byte;
      index: Int32;
      separation: Float;
   end;

function b2EPEdgeSeparation(const v1, v2, n: TVector2; polygonB: Tb2PolygonShape; radius: Float): Tb2EPAxis;
const
	 k_relativeTol = 0.98;
	 k_absoluteTol = 0.001;
var
   i: Integer;
   s, s1, s2: Float;
begin
   // EdgeA separation
   Result.AxisType := e_ep_edgeA;
   Result.index := 0;
   {$IFDEF OP_OVERLOAD}
   Result.separation := b2Dot(n, polygonB.m_vertices[0] - v1);
   {$ELSE}
   Result.separation := b2Dot(n, Subtract(polygonB.m_vertices[0], v1));
   {$ENDIF}
   for i := 0 to polygonB.m_vertexCount - 1 do
   begin
      {$IFDEF OP_OVERLOAD}
      s := b2Dot(n, polygonB.m_vertices[i] - v1);
      {$ELSE}
      s := b2Dot(n, Subtract(polygonB.m_vertices[i], v1));
      {$ENDIF}
      if s < Result.separation then
         Result.separation := s;
   end;
end;

function b2EPPolygonSeparation(const v1, v2, n: TVector2; polygonB: Tb2PolygonShape; radius: Float): Tb2EPAxis;
var
   i: Integer;
   s, s1, s2: Float;
begin
   Result.AxisType := e_ep_edgeB;
   Result.index := 0;
   Result.separation := -FLT_MAX;
   for i := 0 to polygonB.m_vertexCount - 1 do
   begin
      {$IFDEF OP_OVERLOAD}
      s1 := b2Dot(polygonB.m_normals[i], v1 - polygonB.m_vertices[i]);
      s2 := b2Dot(polygonB.m_normals[i], v2 - polygonB.m_vertices[i]);
      {$ELSE}
      s1 := b2Dot(polygonB.m_normals[i], Subtract(v1, polygonB.m_vertices[i]));
      s2 := b2Dot(polygonB.m_normals[i], Subtract(v2, polygonB.m_vertices[i]));
      {$ENDIF}
      s := b2Min(s1, s2);
      if s > Result.separation then
      begin
         Result.index := i;
         Result.separation := s;

         if s > radius then
            Exit;
      end;
   end;
end;

type
   /// Used for computing contact manifolds.
   PClipVertex = ^TClipVertex;
   TClipVertex = record
      v: TVector2;
      id: Tb2ContactID;
   end;

   PClipVertices = ^TClipVertices;
   TClipVertices = array[0..1] of TClipVertex;

procedure b2FindIncidentEdge(var c: TClipVertices; poly1, poly2: Tb2PolygonShape;
   edge1: Int32); overload;
var
   i: Integer;
   index, i1, i2: Int32;
   normal1: TVector2;
   minDot, dot: Float;
begin
   //b2Assert(0 <= edge1 && edge1 < poly1.m_vertexCount);

   // Get the normal of the reference edge in poly2's frame.
   normal1 := poly1.m_normals[edge1];

   // Find the incident edge on poly2.
   index := 0;
   minDot := FLT_MAX;
   for i := 0 to poly2.m_vertexCount - 1 do
   begin
      dot := b2Dot(normal1, poly2.m_normals[i]);
      if dot < minDot then
      begin
         minDot := dot;
         index := i;
      end;
   end;

   // Build the clip vertices for the incident edge.
   i1 := index;
   if i1 + 1 < poly2.m_vertexCount then
      i2 := i1 + 1
   else
      i2 := 0;

   with c[0], id.cf do
   begin
      v := poly2.m_vertices[i1];
      indexA := edge1;
      indexB := i1;
      typeA := e_contact_feature_face;
      typeB := e_contact_feature_vertex;
   end;

   with c[1], id.cf do
   begin
      v := poly2.m_vertices[i2];
      indexA := edge1;
      indexB := i2;
      typeA := e_contact_feature_face;
      typeB := e_contact_feature_vertex;
   end;
end;

/// Clipping for contact manifolds.
function b2ClipSegmentToLine(var vOut: TClipVertices; const vIn: TClipVertices;
   const normal: TVector2; offset: Float; vertexIndexA: Int32): Int32;
var
   distance0, distance1, interp: Float;
begin
   Result := 0; // Start with no output points

   // Calculate the distance of end points to the line
   distance0 := b2Dot(normal, vIn[0].v) - offset;
   distance1 := b2Dot(normal, vIn[1].v) - offset;

   // If the points are behind the plane
   if distance0 <= 0.0 then
   begin
      vOut[Result] := vIn[0];
      Inc(Result);
   end;
   if distance1 <= 0.0 then
   begin
      vOut[Result] := vIn[1];
      Inc(Result);
   end;

   // If the points are on different sides of the plane
   if distance0 * distance1 < 0.0 then
   begin
      // Find intersection point of edge and plane
      interp := distance0 / (distance0 - distance1);
      {$IFDEF OP_OVERLOAD}
      vOut[Result].v := vIn[0].v + interp * (vIn[1].v - vIn[0].v);
      {$ELSE}
      vOut[Result].v := Add(vIn[0].v, Multiply(Subtract(vIn[1].v, vIn[0].v), interp));
      {$ENDIF}

      // VertexA is hitting edgeB.
      with vOut[Result].id.cf do
      begin
         indexA := vertexIndexA;
         indexB := vIn[0].id.cf.indexB;
         typeA := e_contact_feature_vertex;
         typeB := e_contact_feature_face;
      end;
      Inc(Result);
   end;
end;

// Collide and edge and polygon. This uses the SAT and clipping to produce up to 2 contact points.
// Edge adjacency is handle to produce locally valid contact points and normals. This is intended
// to allow the polygon to slide smoothly over an edge chain.
//
// Algorithm
// 1. Classify front-side or back-side collision with edge.
// 2. Compute separation
// 3. Process adjacent edges
// 4. Classify adjacent edge as convex, flat, null, or concave
// 5. Skip null or concave edges. Concave edges get a separate manifold.
// 6. If the edge is flat, compute contact points as normal. Discard boundary points.
// 7. If the edge is convex, compute it's separation.
// 8. Use the minimum separation of up to three edges. If the minimum separation
//    is not the primary edge, return.
// 9. If the minimum separation is the primary edge, compute the contact points and return.
var
   static_polygonshape1, static_polygonshape2: Tb2PolygonShape;
procedure b2CollideEdgeAndPolygon(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
const
   k_relativeTol = 0.98;
   k_absoluteTol = 0.001;
var
   i: Integer;
   xf: Tb2Transform;
   edgeA: Tb2EdgeShape;
   polygonB_in, poly1, poly2: Tb2PolygonShape;
   v1, v2, e, edgeNormal, v0, v3, e0, n0, e2, n2: TVector2;
   isFrontSide: Boolean;
   edgeAxis, axis, polygonAxis, primaryAxis: Tb2EPAxis;
   types: array[0..1] of Tb2EdgeType;
   s, totalRadius, frontOffset, sideOffset1, sideOffset2: Float;
   incidentEdge, clipPoints1, clipPoints2: TClipVertices;
   clipVertex: PClipVertex;
   iv1, iv2, pointCount: Int32;
   tangent: TVector2 absolute e;
   planePoint: TVector2 absolute v0;
begin
   if ABfixture then
   begin
      edgeA := Tb2EdgeShape(Tb2Fixture(A).m_shape);
      polygonB_in := Tb2PolygonShape(Tb2Fixture(B).m_shape);
   end
   else
   begin
      edgeA := Tb2EdgeShape(A);
      polygonB_in := Tb2PolygonShape(B);
   end;

   manifold.pointCount := 0;
   xf := b2MulT(xfA, xfB);

	 // Create a polygon for edge shape A
	 static_polygonshape1.SetAsEdge(edgeA.m_vertex1, edgeA.m_vertex2);

   // Build static_polygonshape2 in frame A
   static_polygonshape2.m_radius := polygonB_in.m_radius;
   static_polygonshape2.m_vertexCount := polygonB_in.m_vertexCount;
   static_polygonshape2.m_centroid := b2Mul(xf, polygonB_in.m_centroid);
   for i := 0 to static_polygonshape2.m_vertexCount - 1 do
   begin
      static_polygonshape2.m_vertices[i] := b2Mul(xf, polygonB_in.m_vertices[i]);
      static_polygonshape2.m_normals[i] := b2Mul(xf.R, polygonB_in.m_normals[i]);
   end;

   totalRadius := static_polygonshape1.m_radius + static_polygonshape2.m_radius;
   // Edge geometry
   v1 := edgeA.m_vertex1;
   v2 := edgeA.m_vertex2;
   {$IFDEF OP_OVERLOAD}
   e := v2 - v1;
   {$ELSE}
   e := Subtract(v2, v1);
   {$ENDIF}
   edgeNormal.x := e.y;
   edgeNormal.y := -e.x;
   {$IFDEF OP_OVERLOAD}
   edgeNormal.Normalize;
   {$ELSE}
   Normalize(edgeNormal);
   {$ENDIF}

   // Determine side
   {$IFDEF OP_OVERLOAD}
   isFrontSide := b2Dot(edgeNormal, static_polygonshape2.m_centroid - v1) >= 0.0;
   {$ELSE}
   isFrontSide := b2Dot(edgeNormal, Subtract(static_polygonshape2.m_centroid, v1)) >= 0.0;
   {$ENDIF}
   if not isFrontSide then
   begin
      // flip normal for backside collision
      {$IFDEF OP_OVERLOAD}
      edgeNormal := -edgeNormal;
      {$ELSE}
      edgeNormal := Negative(edgeNormal);
      {$ENDIF}
   end;

   // Compute primary separating axis
   edgeAxis := b2EPEdgeSeparation(v1, v2, edgeNormal, static_polygonshape2, totalRadius);
   if edgeAxis.separation > totalRadius then
   begin
      // Shapes are separated
      Exit;
   end;

   // Classify adjacent edges
   types[0] := b2_isolated;
   types[1] := b2_isolated;
   if edgeA.m_hasVertex0 then
   begin
      v0 := edgeA.m_vertex0;
      {$IFDEF OP_OVERLOAD}
      s := b2Dot(edgeNormal, v0 - v1);
      {$ELSE}
      s := b2Dot(edgeNormal, Subtract(v0, v1));
      {$ENDIF}
      if s > 0.1 * b2_linearSlop then
         types[0] := b2_concave
      else if s >= -0.1 * b2_linearSlop then
         types[0] := b2_flat
      else
         types[0] := b2_convex;
   end;

   if edgeA.m_hasVertex3 then
   begin
      v3 := edgeA.m_vertex3;
      {$IFDEF OP_OVERLOAD}
      s := b2Dot(edgeNormal, v3 - v2);
      {$ELSE}
      s := b2Dot(edgeNormal, Subtract(v3, v2));
      {$ENDIF}
      if s > 0.1 * b2_linearSlop then
         types[1] := b2_concave
      else if s >= -0.1 * b2_linearSlop then
         types[1] := b2_flat
      else
         types[1] := b2_convex;
   end;

   if types[0] = b2_convex then
   begin
     // Check separation on previous edge.
     v0 := edgeA.m_vertex0;
     {$IFDEF OP_OVERLOAD}
     e0 := v1 - v0;
     {$ELSE}
     e0 := Subtract(v1, v0);
     {$ENDIF}
     n0.x := e0.y;
     n0.y := -e0.x;
     {$IFDEF OP_OVERLOAD}
     n0.Normalize;
     if not isFrontSide then
        n0 := -n0;
     {$ELSE}
     Normalize(n0);
     if not isFrontSide then
        n0 := Negative(n0);
     {$ENDIF}

     axis := b2EPEdgeSeparation(v0, v1, n0, static_polygonshape2, totalRadius);
     if axis.separation > edgeAxis.separation then
        Exit;
   end;

   if types[1] = b2_convex then
   begin
      // Check separation on next edge.
      v3 := edgeA.m_vertex3;
      {$IFDEF OP_OVERLOAD}
      e2 := v3 - v2;
      {$ELSE}
      e2 := Subtract(v3, v2);
      {$ENDIF}
      n2.x := e2.y;
      n2.y := -e2.x;
      {$IFDEF OP_OVERLOAD}
      n2.Normalize;
      if not isFrontSide then
         n2 := -n2;
      {$ELSE}
      Normalize(n2);
      if not isFrontSide then
         n2 := Negative(n2);
      {$ENDIF}

      axis := b2EPEdgeSeparation(v2, v3, n2, static_polygonshape2, totalRadius);
      if axis.separation > edgeAxis.separation then
         Exit; // The polygon should collide with the next edge
   end;

   polygonAxis := b2EPPolygonSeparation(v1, v2, edgeNormal, static_polygonshape2, totalRadius);
   if polygonAxis.separation > totalRadius then
      Exit;

   if polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol then
      primaryAxis := polygonAxis
   else
      primaryAxis := edgeAxis;

   if edgeAxis.AxisType = e_ep_edgeA then
   begin
      poly1 := static_polygonshape1;
      poly2 := static_polygonshape2;
      if not isFrontSide then
         edgeAxis.index := 1;
      manifold.manifoldType := e_manifold_faceA;
   end
   else
   begin
      poly1 := static_polygonshape2;
      poly2 := static_polygonshape1;
      manifold.manifoldType := e_manifold_faceB;
   end;

   b2FindIncidentEdge(incidentEdge, poly1, poly2, edgeAxis.index);

   iv1 := edgeAxis.index;
   if edgeAxis.index + 1 < poly1.m_vertexCount then
      iv2 := edgeAxis.index + 1
   else
      iv2 := 0;

   v1 := poly1.m_vertices[iv1];
   v2 := poly1.m_vertices[iv2];

   {$IFDEF OP_OVERLOAD}
   tangent := v2 - v1;
   tangent.Normalize;
   {$ELSE}
   tangent := Subtract(v2, v1);
   Normalize(tangent);
   {$ENDIF}

   // edgeNormal works as normal variable in C++
   edgeNormal := b2Cross(tangent, 1.0);
   planePoint := b2MiddlePoint(v1, v2);

   // Face offset.
   frontOffset := b2Dot(edgeNormal, v1);

   // Side offsets, extended by polytope skin thickness.
   sideOffset1 := -b2Dot(tangent, v1) + totalRadius;
   sideOffset2 := b2Dot(tangent, v2) + totalRadius;

   // Clip incident edge against extruded edge1 side edges.
   // Clip to box side 1
   {$IFDEF OP_OVERLOAD}
   if b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1) < b2_maxManifoldPoints then
   {$ELSE}
   if b2ClipSegmentToLine(clipPoints1, incidentEdge, Negative(tangent), sideOffset1, iv1) < b2_maxManifoldPoints then
   {$ENDIF}
      Exit;

   // Clip to negative box side 1
   if b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2) < b2_maxManifoldPoints then
      Exit;

   // Now clipPoints2 contains the clipped points.
   if edgeAxis.AxisType = e_ep_edgeA then
   begin
      manifold.localNormal := edgeNormal;
      manifold.localPoint := planePoint;
   end
   else
   begin
      manifold.localNormal := b2MulT(xf.R, edgeNormal);
      manifold.localPoint := b2MulT(xf, planePoint);
   end;

   pointCount := 0;
   for i := 0 to b2_maxManifoldPoints - 1 do
   begin
      clipVertex := @clipPoints2[i];
      if b2Dot(edgeNormal, clipVertex^.v) - frontOffset <= totalRadius then
         with manifold.points[pointCount] do
         begin
            if edgeAxis.AxisType = e_ep_edgeA then
            begin
               localPoint := b2MulT(xf, clipVertex^.v);
               id := clipVertex^.id;
            end
            else
            begin
               localPoint := clipVertex^.v;
               with id.cf, clipVertex^.id do
               begin
                  typeA := cf.typeB; // cf is clipVertex^.id.cf
                  typeB := cf.typeA;
                  indexA := cf.indexB;
                  indexB := cf.indexA;
               end;
            end;

            if (id.cf.typeA = e_contact_feature_vertex) and (types[id.cf.indexA] = b2_flat) then
               Continue;

            Inc(pointCount);
         end;
   end;

   manifold.pointCount := pointCount;
end;

var
   static_edgeshape: Tb2EdgeShape;
procedure b2CollideLoopAndCircle(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
var
   loop: Tb2LoopShape;
   circle: Tb2CircleShape;
begin
   if ABfixture then
   begin
      loop := Tb2LoopShape(Tb2Fixture(A).m_shape);
      circle := Tb2CircleShape(Tb2Fixture(B).m_shape);
   end
   else
   begin
      loop := Tb2LoopShape(A);
      circle := Tb2CircleShape(B);
   end;

   loop.GetChildEdge(static_edgeshape, contact^.m_indexA);
	 b2CollideEdgeAndCircle(contact,	manifold, static_edgeshape, circle, xfA, xfB, False);
end;

procedure b2CollideLoopAndPolygon(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
var
   loop: Tb2LoopShape;
   poly: Tb2PolygonShape;
begin
   if ABfixture then
   begin
      loop := Tb2LoopShape(Tb2Fixture(A).m_shape);
      poly := Tb2PolygonShape(Tb2Fixture(B).m_shape);
   end
   else
   begin
      loop := Tb2LoopShape(A);
      poly := Tb2PolygonShape(B);
   end;

   loop.GetChildEdge(static_edgeshape, contact^.m_indexA);
	 b2CollideEdgeAndPolygon(contact,	manifold, static_edgeshape, poly, xfA, xfB, False);
end;

// Find the separation between poly1 and poly2 for a give edge normal on poly1.
function b2EdgeSeparation(poly1, poly2: Tb2PolygonShape; const xf1,
   xf2: Tb2Transform; edge1: Int32): Float;
var
   i: Integer;
   index: Int32;
   minDot, dot: Float;
   normal1World, normal1: TVector2;
   v1, v2: TVector2;
begin
   //b2Assert(0 <= edge1 && edge1 < count1);

   // Convert normal from poly1's frame into poly2's frame.
   normal1World := b2Mul(xf1.R, poly1.m_normals[edge1]);
   normal1 := b2MulT(xf2.R, normal1World);

   // Find support vertex on poly2 for -normal.
   index := 0;
   minDot := FLT_MAX;

   for i := 0 to poly2.m_vertexCount - 1 do
   begin
      dot := b2Dot(poly2.m_vertices[i], normal1);
      if dot < minDot then
      begin
         minDot := dot;
         index := i;
      end;
   end;

   v1 := b2Mul(xf1, poly1.m_vertices[edge1]);
   v2 := b2Mul(xf2, poly2.m_vertices[index]);
   {$IFDEF OP_OVERLOAD}
   Result := b2Dot(v2 - v1, normal1World);
   {$ELSE}
   Result := b2Dot(Subtract(v2, v1), normal1World);
   {$ENDIF}
end;

// Find the max separation between poly1 and poly2 using edge normals from poly1.
function b2FindMaxSeparation(var edgeIndex: Int32;
   poly1, poly2: Tb2PolygonShape; const xf1, xf2: Tb2Transform): Float;
var
   i: Integer;
   edge, prevEdge, nextEdge, bestEdge, increment: Int32;
   d, dLocal1: TVector2;
   maxDot, dot, s, sPrev, sNext, bestSeparation: Float;
begin
   // Vector pointing from the centroid of poly1 to the centroid of poly2.
   {$IFDEF OP_OVERLOAD}
   d := b2Mul(xf2, poly2.m_centroid) - b2Mul(xf1, poly1.m_centroid);
   {$ELSE}
   d := Subtract(b2Mul(xf2, poly2.m_centroid), b2Mul(xf1, poly1.m_centroid));
   {$ENDIF}
   dLocal1 := b2MulT(xf1.R, d);

   // Find edge normal on poly1 that has the largest projection onto d.
   edge := 0;
   maxDot := -FLT_MAX;
   for i := 0 to poly1.m_vertexCount - 1 do
   begin
      dot := b2Dot(poly1.m_normals[i], dLocal1);
      if dot > maxDot then
      begin
         maxDot := dot;
         edge := i;
      end;
   end;

   // Get the separation for the edge normal.
   s := b2EdgeSeparation(poly1, poly2, xf1, xf2, edge);

   // Check the separation for the previous edge normal.
   if edge - 1 >= 0 then
      prevEdge := edge - 1
   else
      prevEdge := poly1.m_vertexCount - 1;

   sPrev := b2EdgeSeparation(poly1, poly2, xf1, xf2, prevEdge);

   // Check the separation for the next edge normal.
   if edge + 1 < poly1.m_vertexCount then
      nextEdge := edge + 1
   else
      nextEdge := 0;

   sNext := b2EdgeSeparation(poly1, poly2, xf1, xf2, nextEdge);

   // Find the best edge and the search direction.
   if (sPrev > s) and (sPrev > sNext) then
   begin
      increment := -1;
      bestEdge := prevEdge;
      bestSeparation := sPrev;
   end
   else if sNext > s then
   begin
      increment := 1;
      bestEdge := nextEdge;
      bestSeparation := sNext;
   end
   else
   begin
      edgeIndex := edge;
      Result := s;
      Exit;
   end;

   // Perform a local search for the best edge normal.
   while True do
   begin
      if increment = -1 then
      begin
         if bestEdge - 1 >= 0 then
            edge := bestEdge - 1
         else
            edge := poly1.m_vertexCount - 1;
      end
      else
         if bestEdge + 1 < poly1.m_vertexCount then
            edge := bestEdge + 1
         else
            edge := 0;

      s := b2EdgeSeparation(poly1, poly2, xf1, xf2, edge);

      if s > bestSeparation then
      begin
         bestEdge := edge;
         bestSeparation := s;
      end
      else
         Break;
   end;

   edgeIndex := bestEdge;
   Result := bestSeparation;
end;

procedure b2FindIncidentEdge(var c: TClipVertices; poly1, poly2: Tb2PolygonShape;
   const xf1, xf2: Tb2Transform; edge1: Int32); overload;
var
   i: Integer;
   index, i1, i2: Int32;
   normal1: TVector2;
   minDot, dot: Float;
begin
   //b2Assert(0 <= edge1 && edge1 < poly1.m_vertexCount);

   // Get the normal of the reference edge in poly2's frame.
   normal1 := b2MulT(xf2.R, b2Mul(xf1.R, poly1.m_normals[edge1]));

   // Find the incident edge on poly2.
   index := 0;
   minDot := FLT_MAX;
   for i := 0 to poly2.m_vertexCount - 1 do
   begin
      dot := b2Dot(normal1, poly2.m_normals[i]);
      if dot < minDot then
      begin
         minDot := dot;
         index := i;
      end;
   end;

   // Build the clip vertices for the incident edge.
   i1 := index;
   if i1 + 1 < poly2.m_vertexCount then
      i2 := i1 + 1
   else
      i2 := 0;

   with c[0], id.cf do
   begin
      v := b2Mul(xf2, poly2.m_vertices[i1]);
      indexA := edge1;
      indexB := i1;
      typeA := e_contact_feature_face;
      typeB := e_contact_feature_vertex;
   end;

   with c[1], id.cf do
   begin
      v := b2Mul(xf2, poly2.m_vertices[i2]);
      indexA := edge1;
      indexB := i2;
      typeA := e_contact_feature_face;
      typeB := e_contact_feature_vertex;
   end;
end;

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip the normal points from 1 to 2
procedure b2CollidePolygons(contact: Pb2Contact; var manifold: Tb2Manifold;
   A, B: TObject; const xfA, xfB: Tb2Transform; ABfixture: Boolean);
var
   polyA, polyB: Tb2PolygonShape;
   i: Integer;
   edgeA, edgeB: Int32;
   edge1: Int32; // reference edge
   iv1, iv2: Int32;
   flip: UInt8;
   totalRadius, separationA, separationB: Float;
   poly1, poly2: Tb2PolygonShape; // reference poly and incident poly
   xf1, xf2: Tb2Transform;
   k_relativeTol, k_absoluteTol: Float;
   incidentEdge, clipPoints1, clipPoints2: TClipVertices;
   v11, v12, localTangent, localNormal, planePoint, tangent, normal: TVector2;
   frontOffset, sideOffset1, sideOffset2: Float;
   np: Int32; // Clip incident edge against extruded edge1 side edges.
   pointCount: Int32;
   cfSwap: Tb2ContactFeature;
begin
   if ABfixture then
   begin
      polyA := Tb2PolygonShape(Tb2Fixture(A).m_shape);
      polyB := Tb2PolygonShape(Tb2Fixture(B).m_shape);
   end
   else
   begin
      polyA := Tb2PolygonShape(A);
      polyB := Tb2PolygonShape(B);
   end;

   manifold.pointCount := 0;
   totalRadius := polyA.m_radius + polyB.m_radius;

   separationA := b2FindMaxSeparation(edgeA, polyA, polyB, xfA, xfB);
   if separationA > totalRadius then
      Exit;

   edgeB := 0;
   separationB := b2FindMaxSeparation(edgeB, polyB, polyA, xfB, xfA);
   if separationB > totalRadius then
      Exit;

   k_relativeTol := 0.98;
   k_absoluteTol := 0.001;

   if separationB > k_relativeTol * separationA + k_absoluteTol then
   begin
      poly1 := polyB;
      poly2 := polyA;
      xf1 := xfB;
      xf2 := xfA;
      edge1 := edgeB;
      manifold.manifoldType := e_manifold_faceB;
      flip := 1;
   end
   else
   begin
      poly1 := polyA;
      poly2 := polyB;
      xf1 := xfA;
      xf2 := xfB;
      edge1 := edgeA;
      manifold.manifoldType := e_manifold_faceA;
      flip := 0;
   end;

   b2FindIncidentEdge(incidentEdge, poly1, poly2, xf1, xf2, edge1);

   iv1 := edge1;
   if edge1 + 1 < poly1.m_vertexCount then
      iv2 := edge1 + 1
   else
      iv2 := 0;

   v11 := poly1.m_vertices[iv1];
   v12 := poly1.m_vertices[iv2];

   {$IFDEF OP_OVERLOAD}
   localTangent := v12 - v11;
   localTangent.Normalize;
   {$ELSE}
   localTangent := Subtract(v12, v11);
   Normalize(localTangent);
   {$ENDIF}

   localNormal := b2Cross(localTangent, 1.0);
   planePoint := b2MiddlePoint(v11, v12);

   tangent := b2Mul(xf1.R, localTangent);
   normal := b2Cross(tangent, 1.0);

   v11 := b2Mul(xf1, v11);
   v12 := b2Mul(xf1, v12);

   // Face offset.
   frontOffset := b2Dot(normal, v11);

   // Side offsets, extended by polytope skin thickness.
   sideOffset1 := -b2Dot(tangent, v11) + totalRadius;
   sideOffset2 := b2Dot(tangent, v12) + totalRadius;

   // Clip to box side 1
   {$IFDEF OP_OVERLOAD}
   np := b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);
   {$ELSE}
   np := b2ClipSegmentToLine(clipPoints1, incidentEdge, Negative(tangent), sideOffset1, iv1);
   {$ENDIF}
   if np < 2 then
      Exit;

   // Clip to negative box side 1
   np := b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);
   if np < 2 then
      Exit;

   // Now clipPoints2 contains the clipped points.
   manifold.localNormal := localNormal;
   manifold.localPoint := planePoint;

   pointCount := 0;
   for i := 0 to b2_maxManifoldPoints - 1 do
   begin
      if b2Dot(normal, clipPoints2[i].v) - frontOffset <= totalRadius then
         with manifold.points[pointCount] do
         begin
            localPoint := b2MulT(xf2, clipPoints2[i].v);
            id := clipPoints2[i].id;
            if flip <> 0 then
               with id.cf do
               begin
                  // Swap features
                  cfSwap := id.cf;
                  indexA := cfSwap.indexB;
                  indexB := cfSwap.indexA;
                  typeA := cfSwap.typeB;
                  typeB := cfSwap.typeA;
               end;

            Inc(pointCount);
         end;
   end;

   manifold.pointCount := pointCount;
end;

{ Tb2CircleShape }

constructor Tb2CircleShape.Create;
begin
	 m_type := e_circleShape;
   m_radius := 0.0;
   m_p := b2Vec2_Zero;
end;

function Tb2CircleShape.Clone: Tb2Shape;
begin
   Result := Tb2CircleShape.Create;
   Result.m_type := m_type;
   Result.m_radius := m_radius;
   Tb2CircleShape(Result).m_p := m_p;
end;

function Tb2CircleShape.GetChildCount: Int32;
begin
   Result := 1;
end;

function Tb2CircleShape.TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean;
var
   center, d: TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   center := xf.position + b2Mul(xf.R, m_p);
   d := p - center;
   {$ELSE}
   center := Add(xf.position, b2Mul(xf.R, m_p));
   d := Subtract(p, center);
   {$ENDIF}
	 Result := b2Dot(d, d) <= m_radius * m_radius;
end;

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
function Tb2CircleShape.RayCast(var output: Tb2RayCastOutput;
   const input: Tb2RayCastInput; const transform: Tb2Transform;
   childIndex: Int32): Boolean;
var
   s, r: TVector2;
   b, c, rr, sigma, a: Float;
begin
   //B2_NOT_USED(childIndex);
   {$IFDEF OP_OVERLOAD}
   s := input.p1 - (transform.position + b2Mul(transform.R, m_p));
   {$ELSE}
   s := Subtract(input.p1, Add(transform.position, b2Mul(transform.R, m_p)));
   {$ENDIF}
   b := b2Dot(s, s) - m_radius * m_radius;

   // Solve quadratic equation.
   {$IFDEF OP_OVERLOAD}
   r := input.p2 - input.p1;
   {$ELSE}
   r := Subtract(input.p2, input.p1);
   {$ENDIF}
   c :=  b2Dot(s, r);
   rr := b2Dot(r, r);
   sigma := c * c - rr * b;

   // Check for negative discriminant and short segment.
   if (sigma < 0.0) or (rr < FLT_EPSILON) then
   begin
      Result := False;
      Exit;
   end;

   // Find the point of intersection of the line with the circle.
   a := -(c + Sqrt(sigma));

   // Is the intersection point on the segment?
   if (0.0 <= a) and (a <= input.maxFraction * rr) then
   begin
      a := a / rr;
      with output do
      begin
         output.fraction := a;
         {$IFDEF OP_OVERLOAD}
         output.normal := s + a * r;
         output.normal.Normalize;
         {$ELSE}
         output.normal := Add(s, Multiply(r, a));
         Normalize(output.normal);
         {$ENDIF}
      end;
      Result := True;
      Exit;
   end;

   Result := False;
end;

procedure Tb2CircleShape.ComputeAABB(var aabb: Tb2AABB;
   const xf: Tb2Transform; childIndex: Int32);
var
   p: TVector2;
begin
   //B2_NOT_USED(childIndex);
   {$IFDEF OP_OVERLOAD}
   p := xf.position + b2Mul(xf.R, m_p);
   {$ELSE}
   p := Add(xf.position, b2Mul(xf.R, m_p));
   {$ENDIF}
   SetValue(aabb.lowerBound, p.x - m_radius, p.y - m_radius);
   SetValue(aabb.upperBound, p.x + m_radius, p.y + m_radius);
end;

procedure Tb2CircleShape.ComputeMass(var massData: Tb2MassData; density: Float);
begin
   m_baseMass.mass := Pi * m_radius * m_radius;
   m_baseMass.center := m_p;
   m_baseMass.I := m_baseMass.mass * (0.5 * m_radius * m_radius + b2Dot(m_p, m_p));

   massData.mass := density * m_baseMass.mass;
   massData.center := m_p;
   massData.I := density * m_baseMass.I;
end;

function Tb2CircleShape.ComputeSubmergedArea(const normal: TVector2; offset: Float;
   const xf: Tb2Transform; var c: TVector2): Float;
var
   p: TVector2;
   l, r2, l2, com: Float;
begin
   p := b2Mul(xf, m_p);
   l := -(b2Dot(normal, p) - offset);

   if l < -m_radius + FLT_EPSILON then
   begin
      Result := 0.0; // Untouch
      Exit;
   end;

   r2 := m_radius * m_radius;
   if l > m_radius then
   begin
      //Completely wet
      c := p;
      Result := Pi * r2;
      Exit;
   end;

   //Magic
   l2 := l * l;
   Result := r2 *(ArcSin(l / m_radius) + Pi / 2) + l * Sqrt(r2 - l2);
   com := -2 / 3 * Power(r2 - l2, 1.5) / Result;

   c.x := p.x + normal.x * com;
   c.y := p.y + normal.y * com;
end;

function Tb2CircleShape.GetSupport(const d: TVector2): Int32;
begin
   Result := 0;
end;

function Tb2CircleShape.GetSupportVertex(const d: TVector2): TVector2;
begin
   Result := m_p;
end;

function Tb2CircleShape.GetVertexCount: Int32;
begin
   Result := 1;
end;

function Tb2CircleShape.GetVertex(index: Int32): TVector2;
begin
   //b2Assert(index == 0);
   Result := m_p;
end;

{ Tb2PolygonShape }

constructor Tb2PolygonShape.Create;
begin
   m_type := e_polygonShape;
   m_radius := b2_polygonRadius;
   m_vertexCount := 0;
   m_centroid := b2Vec2_Zero;
   m_edgeShapeMassed := False;
end;

function Tb2PolygonShape.Clone: Tb2Shape;
begin
   Result := Tb2PolygonShape.Create;
   Result.m_type := m_type;
   Result.m_radius := m_radius;

   with Tb2PolygonShape(Result) do
   begin
      m_centroid := Self.m_centroid;
      m_vertices := Self.m_vertices;
      m_normals := Self.m_normals;
      m_vertexCount := Self.m_vertexCount;
      m_edgeShapeMassed := Self.m_edgeShapeMassed;
   end;
end;

function Tb2PolygonShape.GetChildCount: Int32;
begin
   Result := 1;
end;

function Tb2PolygonShape.TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean;
var
   i: Integer;
   pLocal: TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   pLocal := b2MulT(xf.R, p - xf.position);
   {$ELSE}
   pLocal := b2MulT(xf.R, Subtract(p, xf.position));
   {$ENDIF}

   for i := 0 to m_vertexCount - 1 do
      {$IFDEF OP_OVERLOAD}
      if b2Dot(m_normals[i], pLocal - m_vertices[i]) > 0.0 then
      {$ELSE}
      if b2Dot(m_normals[i], Subtract(pLocal, m_vertices[i])) > 0.0 then
      {$ENDIF}
      begin
         Result := False;
         Exit;
      end;

   Result := True;
end;

function Tb2PolygonShape.RayCast(var output: Tb2RayCastOutput;
   const input: Tb2RayCastInput; const transform: Tb2Transform;
   childIndex: Int32): Boolean;
var
   i: Integer;
   index: Int32;
   p1, p2, d, q, r: TVector2;
   numerator, denominator, t, rr, s, lower, upper: Float;
begin
   //B2_NOT_USED(childIndex);
   // Put the ray into the polygon's frame of reference.
   {$IFDEF OP_OVERLOAD}
   p1 := b2MulT(transform.R, input.p1 - transform.position);
   p2 := b2MulT(transform.R, input.p2 - transform.position);
   d := p2 - p1;
   {$ELSE}
   p1 := b2MulT(transform.R, Subtract(input.p1, transform.position));
   p2 := b2MulT(transform.R, Subtract(input.p2, transform.position));
   d := Subtract(p2, p1);
   {$ENDIF}

   if m_vertexCount = 2 then
   begin
      //v1 := m_vertices[0];
      //v2 := m_vertices[1];
      //b2Vec2 normal := m_normals[0];

      // q := p1 + t * d
      // dot(normal, q - v1) := 0
      // dot(normal, p1 - v1) + t * dot(normal, d) := 0
      {$IFDEF OP_OVERLOAD}
      numerator := b2Dot(m_normals[0], m_vertices[0] - p1);
      {$ELSE}
      numerator := b2Dot(m_normals[0], Subtract(m_vertices[0], p1));
      {$ENDIF}
      denominator := b2Dot(m_normals[0], d);

      if denominator = 0.0 then
      begin
         Result := False;
         Exit;
      end;

      t := numerator / denominator;
      if (t < 0.0) or (1.0 < t) then
      begin
         Result := False;
         Exit;
      end;

      {$IFDEF OP_OVERLOAD}
      q := p1 + t * d;
      {$ELSE}
      q := Add(p1, Multiply(d, t));
      {$ENDIF}

      // q := v1 + s * r
      // s := dot(q - v1, r) / dot(r, r)
      {$IFDEF OP_OVERLOAD}
      r := m_vertices[1] - m_vertices[0];
      {$ELSE}
      r := Subtract(m_vertices[1], m_vertices[0]);
      {$ENDIF}
      rr := b2Dot(r, r);
      if rr = 0.0 then
      begin
         Result := False;
         Exit;
      end;

      {$IFDEF OP_OVERLOAD}
      s := b2Dot(q - m_vertices[0], r) / rr;
      {$ELSE}
      s := b2Dot(Subtract(q, m_vertices[0]), r) / rr;
      {$ENDIF}
      if (s < 0.0) or (1.0 < s) then
      begin
         Result := False;
         Exit;
      end;

      output.fraction := t;
      if numerator > 0.0 then
         {$IFDEF OP_OVERLOAD}
         output.normal := -m_normals[0]
         {$ELSE}
         output.normal := Negative(m_normals[0])
         {$ENDIF}
      else
         output.normal := m_normals[0];
      Result := True;
      Exit;
   end
   else
   begin
      lower := 0.0;
      upper := input.maxFraction;

      index := -1;

      for i := 0 to m_vertexCount - 1 do
      begin
         // p := p1 + a * d
         // dot(normal, p - v) := 0
         // dot(normal, p1 - v) + a * dot(normal, d) := 0
         {$IFDEF OP_OVERLOAD}
         numerator := b2Dot(m_normals[i], m_vertices[i] - p1);
         {$ELSE}
         numerator := b2Dot(m_normals[i], Subtract(m_vertices[i], p1));
         {$ENDIF}
         denominator := b2Dot(m_normals[i], d);

         if denominator = 0.0 then
         begin
            if numerator < 0.0 then
            begin
               Result := False;
               Exit;
            end;
         end
         else
         begin
            // Note: we want this predicate without division:
            // lower < numerator / denominator, where denominator < 0
            // Since denominator < 0, we have to flip the inequality:
            // lower < numerator / denominator <==> denominator * lower > numerator.
            if (denominator < 0.0) and (numerator < lower * denominator) then
            begin
               // Increase lower.
               // The segment enters this half-space.
               lower := numerator / denominator;
               index := i;
            end
            else if (denominator > 0.0) and (numerator < upper * denominator) then
            begin
               // Decrease upper.
               // The segment exits this half-space.
               upper := numerator / denominator;
            end;
         end;

         // The use of epsilon here causes the assert on lower to trip
         // in some cases. Apparently the use of epsilon was to make edge
         // shapes work, but now those are handled separately.
         //if (upper < lower - b2_epsilon)
         if upper < lower then
         begin
            Result := False;
            Exit;
         end;
      end;

      //b2Assert(0.0f <= lower && lower <= input.maxFraction);

      if index >= 0 then
      begin
         output.fraction := lower;
         output.normal := b2Mul(transform.R, m_normals[index]);
         Result := True;
         Exit;
      end;
   end;

   Result := False;
end;

procedure Tb2PolygonShape.ComputeAABB(var aabb: Tb2AABB;
   const xf: Tb2Transform; childIndex: Int32);
var
   i: Integer;
   lower, upper, v, r: TVector2;
begin
   //B2_NOT_USED(childIndex);
   lower := b2Mul(xf, m_vertices[0]);
   upper := lower;

   for i := 1 to m_vertexCount - 1 do
   begin
      v := b2Mul(xf, m_vertices[i]);
      lower := b2Min(lower, v);
      upper := b2Max(upper, v);
   end;

   SetValue(r, m_radius, m_radius);
   {$IFDEF OP_OVERLOAD}
   aabb.lowerBound := lower - r;
   aabb.upperBound := upper + r;
   {$ELSE}
   aabb.lowerBound := Subtract(lower, r);
   aabb.upperBound := Add(upper, r);
   {$ENDIF}
end;

procedure Tb2PolygonShape.ComputeMass(var massData: Tb2MassData; density: Float);
const
   k_inv3 = 1.0 / 3.0;
var
   i: Integer;
   center, pRef, p1, p2, p3, e1, e2: TVector2;
   area, inertia: Float;
   D, triangleArea: Float;
   px, py, ex1, ey1, ex2, ey2: Float;
   intx2, inty2: Float;
begin
   // Polygon mass, centroid, and inertia.
   // Let rho be the polygon density in mass per unit area.
   // Then:
   // mass = rho * int(dA)
   // centroid.x = (1/mass) * rho * int(x * dA)
   // centroid.y = (1/mass) * rho * int(y * dA)
   // I = rho * int((x*x + y*y) * dA)
   //
   // We can compute these integrals by summing all the integrals
   // for each triangle of the polygon. To evaluate the integral
   // for a single triangle, we make a change of variables to
   // the (u,v) coordinates of the triangle:
   // x = x0 + e1x * u + e2x * v
   // y = y0 + e1y * u + e2y * v
   // where 0 <= u && 0 <= v && u + v <= 1.
   //
   // We integrate u from [0,1-v] and then v from [0,1].
   // We also need to use the Jacobian of the transformation:
   // D = cross(e1, e2)
   //
   // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
   //
   // The rest of the derivation is handled by computer algebra.

   //b2Assert(m_vertexCount >= 3);

   // A line segment has zero mass.
   if m_vertexCount = 2 then
   begin
      massData.center := b2MiddlePoint(m_vertices[0], m_vertices[1]);
      if m_edgeShapeMassed then
      begin
         {$IFDEF OP_OVERLOAD}
         area := (m_vertices[0] - m_vertices[1]).Length;
         {$ELSE}
         area := LengthVec(Subtract(m_vertices[0], m_vertices[1]));
         {$ENDIF}
         massData.mass := density * area;
         massData.I := massData.mass * Sqr(area) / 12;
      end
      else
      begin
         massData.mass := 0.0;
         massData.I := 0.0;
      end;
      Exit;
   end;

   center := b2Vec2_Zero;
   area := 0.0;
   inertia := 0.0;

   // pRef is the reference point for forming triangles.
   // It's location doesn't change the result (except for rounding error).
   pRef := b2Vec2_Zero;

   for i := 0 to m_vertexCount - 1 do
   begin
      // Triangle vertices.
      p1 := pRef;
      p2 := m_vertices[i];
      if (i + 1 < m_vertexCount) then
         p3 := m_vertices[i + 1]
      else
         p3 := m_vertices[0];

      {$IFDEF OP_OVERLOAD}
      e1 := p2 - p1;
      e2 := p3 - p1;
      {$ELSE}
      e1 := Subtract(p2, p1);
      e2 := Subtract(p3, p1);
      {$ENDIF}

      D := b2Cross(e1, e2);

      triangleArea := 0.5 * D;
      area := area + triangleArea;

      // Area weighted centroid
      {$IFDEF OP_OVERLOAD}
      center.AddBy(triangleArea * k_inv3 * (p1 + p2 + p3));
      {$ELSE}
      AddBy(center, Multiply(Add(p1, p2, p3), triangleArea * k_inv3));
      {$ENDIF}

      px := p1.x;
      py := p1.y;
      ex1 := e1.x;
      ey1 := e1.y;
      ex2 := e2.x;
      ey2 := e2.y;

      intx2 := k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) +
         (px * ex1 + px * ex2)) + 0.5 * px * px;
      inty2 := k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) +
         (py * ey1 + py * ey2)) + 0.5 * py * py;

      inertia := inertia + D * (intx2 + inty2);
   end;

   m_baseMass.mass := area;
   m_baseMass.I := inertia;

   // Total mass
   massData.mass := density * area;

   // Center of mass
   //b2Assert(area > B2_FLT_EPSILON);
   {$IFDEF OP_OVERLOAD}
   center.DivideBy(area);
   {$ELSE}
   DivideBy(center, area);
   {$ENDIF}
   massData.center := center;
   m_baseMass.center := center;

   // Inertia tensor relative to the local origin.
   massData.I := density * inertia;
end;

function Tb2PolygonShape.ComputeSubmergedArea(const normal: TVector2; offset: Float;
   const xf: Tb2Transform; var c: TVector2): Float;
var
   i: Integer;
   normalL, intoVec, outoVec, center, p2, p3: TVector2;
   offsetL, intoLamdda, outoLamdda, triangleArea: Float;
   diveCount, intoIndex, outoIndex, intoIndex2, outoIndex2: Int32;
   lastSubmerged, isSubmerged: Boolean;
   depths: array[0..b2_maxPolygonVertices - 1] of Float;
begin
   // Transform plane into shape co-ordinates
   normalL := b2MulT(xf.R, normal);
   offsetL := offset - b2Dot(normal, xf.position);

   diveCount := 0;
   intoIndex := -1;
   outoIndex := -1;
   lastSubmerged := False;

   for i := 0 to m_vertexCount - 1 do
   begin
      depths[i] := b2Dot(normalL, m_vertices[i]) - offsetL;
      isSubmerged := depths[i] < -FLT_EPSILON;
      if i > 0 then
      begin
         if isSubmerged then
         begin
            if not lastSubmerged then
            begin
               intoIndex := i - 1;
               Inc(diveCount);
            end;
         end
         else
         begin
            if lastSubmerged then
            begin
               outoIndex := i - 1;
               Inc(diveCount);
            end;
         end;
      end;
      lastSubmerged := isSubmerged;
   end;

   case diveCount of
      0:
         begin
            if lastSubmerged then
            begin
               // Completely submerged
               c := b2Mul(xf, m_baseMass.center);
               Result := m_baseMass.mass;
            end
            else
               Result := 0; //Completely dry
            Exit;
         end;
      1:
         if intoIndex = -1 then
            intoIndex := m_vertexCount - 1
         else
            outoIndex := m_vertexCount - 1;
   end;

   intoIndex2 := (intoIndex + 1) mod m_vertexCount;
   outoIndex2 := (outoIndex + 1) mod m_vertexCount;
   intoLamdda := -depths[intoIndex] / (depths[intoIndex2] - depths[intoIndex]);
   outoLamdda := -depths[outoIndex] / (depths[outoIndex2] - depths[outoIndex]);

   intoVec.x := m_vertices[intoIndex].x * (1 - intoLamdda) + m_vertices[intoIndex2].x * intoLamdda;
   intoVec.y := m_vertices[intoIndex].y * (1 - intoLamdda) + m_vertices[intoIndex2].y * intoLamdda;
   outoVec.x := m_vertices[outoIndex].x * (1 - outoLamdda) + m_vertices[outoIndex2].x * outoLamdda;
   outoVec.y := m_vertices[outoIndex].y * (1 - outoLamdda) + m_vertices[outoIndex2].y * outoLamdda;

   // Initialize accumulator
   Result := 0;
   center := b2Vec2_Zero;
   p2 := m_vertices[intoIndex2];

   // An awkward loop from intoIndex2+1 to outIndex2
   i := intoIndex2;
   while i <> outoIndex2 do
   begin
      i := (i + 1) mod m_vertexCount;
      if i = outoIndex2 then
         p3 := outoVec
      else
         p3 := m_vertices[i];

      triangleArea := 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) -
         (p2.y - intoVec.y) * (p3.x - intoVec.x));
      Result := Result + triangleArea;
      // Area weighted centroid
      center.x := center.x + triangleArea * (intoVec.x + p2.x + p3.x) / 3;
      center.y := center.y + triangleArea * (intoVec.y + p2.y + p3.y) / 3;

      p2 := p3;
   end;

   //Normalize and transform centroid
   {$IFDEF OP_OVERLOAD}
   center.DivideBy(Result);
   {$ELSE}
   DivideBy(center, Result);
   {$ENDIF}
   c := b2Mul(xf, center);
end;

procedure Tb2PolygonShape.SetVertices(vertices: PVector2; count: Int32);
var
   i: Integer;
   edge: TVector2;
   pv: PVector2;
begin
   //b2Assert(2 <= count && count <= b2_maxPolygonVertices);
   m_vertexCount := count;

   // Copy vertices.
   pv := vertices;
   for i := 0 to m_vertexCount - 1 do
   begin
      m_vertices[i] := pv^;
      Inc(pv);
   end;

   // Compute normals. Ensure the edges have non-zero length.
   for i := 0 to m_vertexCount - 1 do
   begin
      {$IFDEF OP_OVERLOAD}
      if i + 1 < m_vertexCount then
         edge := m_vertices[i + 1] - m_vertices[i]
      else
         edge := m_vertices[0] - m_vertices[i];
      {$ELSE}
      if i + 1 < m_vertexCount then
         edge := Subtract(m_vertices[i + 1], m_vertices[i])
      else
         edge := Subtract(m_vertices[0], m_vertices[i]);
      {$ENDIF}
      //b2Assert(edge.SqrLength > FLT_EPLISON * FLT_EPLISON);
      m_normals[i] := b2Cross(edge, 1.0);
      {$IFDEF OP_OVERLOAD}
      m_normals[i].Normalize;
      {$ELSE}
      Normalize(m_normals[i]);
      {$ENDIF}
   end;

   // Compute the polygon centroid.
   m_centroid := ComputeCentroid(m_vertices, m_vertexCount);
end;

procedure Tb2PolygonShape.SetAsBox(hx, hy: Float);
begin
   m_vertexCount := 4;
   SetValue(m_vertices[0], -hx, -hy);
   SetValue(m_vertices[1], hx, -hy);
   SetValue(m_vertices[2], hx, hy);
   SetValue(m_vertices[3], -hx, hy);
   SetValue(m_normals[0], 0.0, -1.0);
   SetValue(m_normals[1], 1.0, 0.0);
   SetValue(m_normals[2], 0.0, 1.0);
   SetValue(m_normals[3], -1.0, 0.0);
   m_centroid := b2Vec2_Zero;
end;

procedure Tb2PolygonShape.SetAsBox(hx, hy: Float; const center: TVector2; angle: Float);
var
   i: Integer;
   xf: Tb2Transform;
begin
   SetAsBox(hx, hy);
   m_centroid := center;

   xf.position := center;
   {$IFDEF OP_OVERLOAD}
   xf.R.SetValue(angle);
   {$ELSE}
   SetValue(xf.R, angle);
   {$ENDIF}

   // Transform vertices and normals.
   for i := 0 to m_vertexCount - 1 do
   begin
      m_vertices[i] := b2Mul(xf, m_vertices[i]);
      m_normals[i] := b2Mul(xf.R, m_normals[i]);
   end;
end;

procedure Tb2PolygonShape.SetAsEdge(const v1, v2: TVector2);
begin
   m_vertexCount := 2;
   m_vertices[0] := v1;
   m_vertices[1] := v2;
   m_centroid := b2MiddlePoint(v1, v2);
   {$IFDEF OP_OVERLOAD}
   m_normals[0] := b2Cross(v2 - v1, 1.0);
   m_normals[0].Normalize;
   m_normals[1] := -m_normals[0];
   {$ELSE}
   m_normals[0] := b2Cross(Subtract(v2, v1), 1.0);
   Normalize(m_normals[0]);
   m_normals[1] := Negative(m_normals[0]);
   {$ENDIF}
end;

{function Tb2PolygonShape.GetSupport(const d: TVector2): Int32;
var
   i: Integer;
   bestValue, value: Float;
begin
   Result := 0;
   bestValue := b2Dot(m_vertices[0], d);
   for i := 1 to m_vertexCount - 1 do
   begin
      value := b2Dot(m_vertices[i], d);
      if value > bestValue then
      begin
         Result := i;
         bestValue := value;
      end;
   end;
end;

function Tb2PolygonShape.GetSupportVertex(const d: TVector2): PVector2;
var
   i: Integer;
   bestIndex: Int32;
   bestValue, value: Float;
begin
   bestIndex := 0;
   bestValue := b2Dot(m_vertices[0], d);
   for i := 1 to m_vertexCount - 1 do
   begin
      value := b2Dot(m_vertices[i], d);
      if value > bestValue then
      begin
         bestIndex := i;
         bestValue := value;
      end;
   end;
   Result := @m_vertices[bestIndex];
end;}

{ Tb2EdgeShape }

constructor Tb2EdgeShape.Create;
begin
   m_type := e_edgeShape;
   m_radius := b2_polygonRadius;
	 m_hasVertex0 := False;
   m_hasVertex3 := False;
end;

procedure Tb2EdgeShape.SetVertices(const v1, v2: TVector2);
begin
   m_vertex1 := v1;
   m_vertex2 := v2;
   m_hasVertex0 := False;
   m_hasVertex3 := False;
end;

function Tb2EdgeShape.Clone: Tb2Shape;
begin
   Result := Tb2EdgeShape.Create;
   Result.m_type := m_type;
   Result.m_radius := m_radius;

   with Tb2EdgeShape(Result) do
   begin
      m_vertex1 := Self.m_vertex1;
      m_vertex2 := Self.m_vertex2;
      m_hasVertex0 := Self.m_hasVertex0;
      m_hasVertex3 := Self.m_hasVertex3;
   end;
end;

function Tb2EdgeShape.GetChildCount: Int32;
begin
   Result := 1;
end;

function Tb2EdgeShape.TestPoint(const xf: Tb2Transform; const p: TVector2): Boolean;
begin
   //B2_NOT_USED(xf);
   //B2_NOT_USED(p);
   Result := False;
end;

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
function Tb2EdgeShape.RayCast(var output: Tb2RayCastOutput;
   const input: Tb2RayCastInput; const transform: Tb2Transform;
   childIndex: Int32): Boolean;
var
   p1, p2, d, e, normal, q: TVector2;
   numerator, denominator, t, rr, s: Float;
begin
   //B2_NOT_USED(childIndex);
   {$IFDEF OP_OVERLOAD}
   // Put the ray into the edge's frame of reference.
   p1 := b2MulT(transform.R, input.p1 - transform.position);
   p2 := b2MulT(transform.R, input.p2 - transform.position);
   d := p2 - p1;

   e := m_vertex2 - m_vertex1;
   normal.x := e.y;
   normal.y := -e.x;
   normal.Normalize;

   // q := p1 + t * d
   // dot(normal, q - m_vertex1) := 0
   // dot(normal, p1 - m_vertex1) + t * dot(normal, d) := 0
   numerator := b2Dot(normal, m_vertex1 - p1);
   {$ELSE}
   // Put the ray into the edge's frame of reference.
   p1 := b2MulT(transform.R, Subtract(input.p1, transform.position));
   p2 := b2MulT(transform.R, Subtract(input.p2, transform.position));
   d := Subtract(p2, p1);

   e := Subtract(m_vertex2, m_vertex1);
   normal.x := e.y;
   normal.y := -e.x;
   Normalize(normal);

   // q := p1 + t * d
   // dot(normal, q - m_vertex1) := 0
   // dot(normal, p1 - m_vertex1) + t * dot(normal, d) := 0
   numerator := b2Dot(normal, Subtract(m_vertex1, p1));
   {$ENDIF}

   denominator := b2Dot(normal, d);

   if denominator = 0.0 then
   begin
      Result := False;
      Exit;
   end;

   t := numerator / denominator;
   if (t < 0.0) or (1.0 < t) then
   begin
      Result := False;
      Exit;
   end;

   {$IFDEF OP_OVERLOAD}
   q := p1 + t * d;
   {$ELSE}
   q := Add(p1, Multiply(d, t));
   {$ENDIF}

   // q := m_vertex1 + s * r
   // s := dot(q - m_vertex1, r) / dot(r, r)
   rr := b2Dot(e, e);
   if rr = 0.0 then
   begin
      Result := False;
      Exit;
   end;

   {$IFDEF OP_OVERLOAD}
   s := b2Dot(q - m_vertex1, e) / rr;
   {$ELSE}
   s := b2Dot(Subtract(q, m_vertex1), e) / rr;
   {$ENDIF}
   if (s < 0.0) or (1.0 < s) then
   begin
      Result := False;
      Exit;
   end;

   output.fraction := t;
   if numerator > 0.0 then
      {$IFDEF OP_OVERLOAD}
      output.normal := -normal
      {$ELSE}
      output.normal := Negative(normal)
      {$ENDIF}
   else
      output.normal := normal;

   Result := True;
end;

procedure Tb2EdgeShape.ComputeAABB(var aabb: Tb2AABB;
   const xf: Tb2Transform; childIndex: Int32);
var
   v1, v2, lower, upper, r: TVector2;
begin
   //B2_NOT_USED(childIndex);
   v1 := b2Mul(xf, m_vertex1);
   v2 := b2Mul(xf, m_vertex2);

   lower := b2Min(v1, v2);
   upper := b2Max(v1, v2);

   SetValue(r, m_radius, m_radius);
   {$IFDEF OP_OVERLOAD}
   aabb.lowerBound := lower - r;
   aabb.upperBound := upper + r;
   {$ELSE}
   aabb.lowerBound := Subtract(lower, r);
   aabb.upperBound := Add(upper, r);
   {$ENDIF}
end;

procedure Tb2EdgeShape.ComputeMass(var massData: Tb2MassData; density: Float);
begin
   //B2_NOT_USED(density);
   massData.mass := 0.0;
   massData.center := b2MiddlePoint(m_vertex1, m_vertex2);
   massData.I := 0.0;
end;

function Tb2EdgeShape.ComputeSubmergedArea(const normal: TVector2;
   offset: Float; const xf: Tb2Transform; var c: TVector2): Float;
begin
   Result := 0.0;
end;

{ Tb2LoopShape }

constructor Tb2LoopShape.Create;
begin
   m_type := e_loopShape;
   m_radius := b2_polygonRadius;
   m_vertices := nil;
   m_count := 0;
end;

procedure Tb2LoopShape.SetVertices(pv: PVector2; count: Int32);
var
   i: Integer;
begin
   m_count := count;
   SetLength(m_vertices, count);
   for i := 0 to count - 1 do
   begin
      m_vertices[i] := pv^;
      Inc(pv);
   end;
end;

procedure Tb2LoopShape.GetChildEdge(edge: Tb2EdgeShape; index: Int32);
var
   i0, i1, i2, i3: Int32;
begin
   //b2Assert(2 <= m_count);
   //b2Assert(0 <= index && index < m_count);
   edge.m_type := e_edgeShape;
   edge.m_radius := m_radius;
   edge.m_hasVertex0 := True;
   edge.m_hasVertex3 := True;

   if index - 1 >= 0 then
      i0 := index - 1
   else
      i0 := m_count - 1;
   i1 := index;
   if index + 1 < m_count then
      i2 := index + 1
   else
      i2 := 0;
   i3 := index + 2;
   while i3 >= m_count do
      Dec(i3, m_count);

   edge.m_vertex0 := m_vertices[i0];
   edge.m_vertex1 := m_vertices[i1];
   edge.m_vertex2 := m_vertices[i2];
   edge.m_vertex3 := m_vertices[i3];
end;

function Tb2LoopShape.Clone: Tb2Shape;
begin
   Result := Tb2LoopShape.Create;
   Result.m_type := m_type;
   Result.m_radius := m_radius;

   with Tb2LoopShape(Result) do
   begin
      m_count := Self.m_count;
      SetLength(m_vertices, m_count);
      Move(Self.m_vertices[0], m_vertices[0], SizeOf(TVector2) * m_count);
   end;
end;

function Tb2LoopShape.GetChildCount: Int32;
begin
   Result := m_count;
end;

function Tb2LoopShape.TestPoint(const xf: Tb2Transform;
   const p: TVector2): Boolean;
begin
	 //B2_NOT_USED(xf);
	 //B2_NOT_USED(p);
   Result := False;
end;

function Tb2LoopShape.RayCast(var output: Tb2RayCastOutput;
   const input: Tb2RayCastInput; const transform: Tb2Transform;
   childIndex: Int32): Boolean;
var
   i2: Int32;
begin
   //b2Assert(childIndex < m_count);
   i2 := childIndex + 1;
   if i2 = m_count then
      i2 := 0;

   static_edgeshape.m_vertex1 := m_vertices[childIndex];
   static_edgeshape.m_vertex2 := m_vertices[i2];

   Result := static_edgeshape.RayCast(output, input, transform, 0);
end;

procedure Tb2LoopShape.ComputeAABB(var aabb: Tb2AABB; const xf: Tb2Transform;
   childIndex: Int32);
var
   i2: Int32;
   v1, v2: TVector2;
begin
   //b2Assert(childIndex < m_count);

   i2 := childIndex + 1;
   if i2 = m_count then
      i2 := 0;

   v1 := b2Mul(xf, m_vertices[childIndex]);
   v2 := b2Mul(xf, m_vertices[i2]);

   aabb.lowerBound := b2Min(v1, v2);
   aabb.upperBound := b2Max(v1, v2);
end;

procedure Tb2LoopShape.ComputeMass(var massData: Tb2MassData; density: Float);
begin
   //B2_NOT_USED(density);
   massData.mass := 0.0;
   massData.center := b2Vec2_Zero;
   massData.I := 0.0;
end;

function Tb2LoopShape.ComputeSubmergedArea(const normal: TVector2;
   offset: Float; const xf: Tb2Transform; var c: TVector2): Float;
begin
   Result := 0.0;
end;

{ Tb2DistanceJointDef }

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

constructor Tb2DistanceJointDef.Create;
begin
   inherited;
   JointType := e_distanceJoint;
   localAnchorA := b2Vec2_Zero;
   localAnchorB := b2Vec2_Zero;
   length := 1.0;
   frequencyHz := 0.0;
   dampingRatio := 0.0;
end;

procedure Tb2DistanceJointDef.Initialize(bodyA, bodyB: Tb2Body; const anchorA,
   anchorB: TVector2);
begin
   Self.bodyA := bodyA;
   Self.bodyB := bodyB;
   localAnchorA := bodyA.GetLocalPoint(anchorA);
   localAnchorB := bodyB.GetLocalPoint(anchorB);
   {$IFDEF OP_OVERLOAD}
   length := (anchorB - anchorA).Length;
   {$ELSE}
   length := UPhysics2DTypes.LengthVec(Subtract(anchorB, anchorA));
   {$ENDIF}
end;

{ Tb2DistanceJoint }

constructor Tb2DistanceJoint.Create(def: Tb2DistanceJointDef);
begin
   inherited Create(def);
   m_localAnchor1 := def.localAnchorA;
   m_localAnchor2 := def.localAnchorB;
   m_length := def.length;
   m_frequencyHz := def.frequencyHz;
   m_dampingRatio := def.dampingRatio;
   m_impulse := 0.0;
   m_gamma := 0.0;
   m_bias := 0.0;
end;

function Tb2DistanceJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchor1);
end;

function Tb2DistanceJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchor2);
end;

function Tb2DistanceJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := (inv_dt * m_impulse) * m_u;
   {$ELSE}
   Result := Multiply(m_u, inv_dt * m_impulse);
   {$ENDIF}
end;

function Tb2DistanceJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := 0.0;
end;

procedure Tb2DistanceJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   r1, r2, P: TVector2;
   length, cr1u, cr2u, invMass: Float;
   C, omega, d, k: Float;
begin
   // Compute the effective mass matrix.
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   m_u := m_bodyB.m_sweep.c + r2 - m_bodyA.m_sweep.c - r1;
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   m_u := Subtract(r2, r1);
   AddBy(m_u, m_bodyB.m_sweep.c);
   SubtractBy(m_u, m_bodyA.m_sweep.c);
   {$ENDIF}

   // Handle singularity.
   {$IFDEF OP_OVERLOAD}
   length := m_u.Length;
   if length > b2_linearSlop then
      m_u := m_u / length
   else
      m_u := b2Vec2_Zero;
   {$ELSE}
   length := UPhysics2DTypes.LengthVec(m_u);
   if length > b2_linearSlop then
      DivideBy(m_u, length)
   else
      m_u := b2Vec2_Zero;
   {$ENDIF}

   cr1u := b2Cross(r1, m_u);
   cr2u := b2Cross(r2, m_u);
   invMass := m_bodyA.m_invMass + m_bodyA.m_invI * cr1u * cr1u +
      m_bodyB.m_invMass + m_bodyB.m_invI * cr2u * cr2u;

   if invMass <> 0.0 then
      m_mass := 1.0 / invMass
   else
      m_mass := 0.0;

   if m_frequencyHz > 0.0 then
   begin
      C := length - m_length;
      omega := 2.0 * Pi * m_frequencyHz; // Frequency
      d := 2.0 * m_mass * m_dampingRatio * omega; // Damping coefficient
      k := m_mass * omega * omega; // Spring stiffness
      m_gamma := step.dt * (d + step.dt * k); // magic formulas
      if m_gamma <> 0.0 then
         m_gamma := 1.0 / m_gamma
      else
         m_gamma := 0.0;
      m_bias := C * step.dt * k * m_gamma;
      m_mass := invMass + m_gamma;
      if m_mass <> 0.0 then
         m_mass := 1.0 / m_mass
      else
         m_mass := 0.0;
   end;

   if step.warmStarting then
   begin
      // Scale the impulse to support a variable time step.
      m_impulse := m_impulse * step.dtRatio;
      {$IFDEF OP_OVERLOAD}
      P := m_impulse * m_u;
      m_bodyA.m_linearVelocity.SubtractBy(m_bodyA.m_invMass * P);
      m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * P);
      {$ELSE}
      P := Multiply(m_u, m_impulse);
      SubtractBy(m_bodyA.m_linearVelocity, Multiply(P, m_bodyA.m_invMass));
      AddBy(m_bodyB.m_linearVelocity, Multiply(P, m_bodyB.m_invMass));
      {$ENDIF}
      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity - m_bodyA.m_invI * b2Cross(r1, P);
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * b2Cross(r2, P);
   end
   else
      m_impulse := 0.0;
end;

procedure Tb2DistanceJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   r1, r2, v1, v2, P: TVector2;
   Cdot, impulse: Float;
begin
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);

   // Cdot = dot(u, v + cross(w, r))
   v1 := m_bodyA.m_linearVelocity + b2Cross(m_bodyA.m_angularVelocity, r1);
   v2 := m_bodyB.m_linearVelocity + b2Cross(m_bodyB.m_angularVelocity, r2);
   Cdot := b2Dot(m_u, v2 - v1);
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));

   // Cdot = dot(u, v + cross(w, r))
   v1 := Add(m_bodyA.m_linearVelocity, b2Cross(m_bodyA.m_angularVelocity, r1));
   v2 := Add(m_bodyB.m_linearVelocity, b2Cross(m_bodyB.m_angularVelocity, r2));
   Cdot := b2Dot(m_u, Subtract(v2, v1));
   {$ENDIF}

   impulse := -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
   m_impulse := m_impulse + impulse;

   {$IFDEF OP_OVERLOAD}
   P := impulse * m_u;
   m_bodyA.m_linearVelocity.SubtractBy(m_bodyA.m_invMass * P);
   m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * P);
   {$ELSE}
   P := Multiply(m_u, impulse);
   SubtractBy(m_bodyA.m_linearVelocity, Multiply(P, m_bodyA.m_invMass));
   AddBy(m_bodyB.m_linearVelocity, Multiply(P, m_bodyB.m_invMass));
   {$ENDIF}
   m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity - m_bodyA.m_invI * b2Cross(r1, P);
   m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * b2Cross(r2, P);
end;

function Tb2DistanceJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
var
   r1, r2, d, P: TVector2;
   C, impulse: Float;
begin
   if m_frequencyHz > 0.0 then
   begin
      // There is no position correction for soft distance constraints.
      Result := True;
      Exit;
   end;

   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   d := m_bodyB.m_sweep.c + r2 - m_bodyA.m_sweep.c - r1;
   C := d.Normalize - m_length;
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   d := Subtract(r2, r1);
   AddBy(d, m_bodyB.m_sweep.c);
   SubtractBy(d, m_bodyA.m_sweep.c);
   C := Normalize(d) - m_length;
   {$ENDIF}
   C := b2Clamp(C, -b2_maxLinearCorrection, b2_maxLinearCorrection);

   impulse := -m_mass * C;
   m_u := d;
   {$IFDEF OP_OVERLOAD}
   P := impulse * m_u;

   m_bodyA.m_sweep.c.SubtractBy(m_bodyA.m_invMass * P);
   m_bodyB.m_sweep.c.AddBy(m_bodyB.m_invMass * P);
   {$ELSE}
   P := Multiply(m_u, impulse);

   SubtractBy(m_bodyA.m_sweep.c, Multiply(P, m_bodyA.m_invMass));
   AddBy(m_bodyB.m_sweep.c, Multiply(P, m_bodyB.m_invMass));
   {$ENDIF}
   m_bodyA.m_sweep.a := m_bodyA.m_sweep.a - m_bodyA.m_invI * b2Cross(r1, P);
   m_bodyB.m_sweep.a := m_bodyB.m_sweep.a + m_bodyB.m_invI * b2Cross(r2, P);

   m_bodyA.SynchronizeTransform;
   m_bodyB.SynchronizeTransform;

   Result := Abs(C) < b2_linearSlop;
end;

{ Tb2PrismaticJointDef }

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)


// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

constructor Tb2PrismaticJointDef.Create;
begin
   inherited;
   JointType := e_prismaticJoint;
   localAnchorA := b2Vec2_Zero;
   localAnchorB := b2Vec2_Zero;
   SetValue(localAxis1, 1.0, 0.0);
   referenceAngle := 0.0;
   enableLimit := False;
   lowerTranslation := 0.0;
   upperTranslation := 0.0;
   enableMotor := False;
   maxMotorForce := 0.0;
   motorSpeed := 0.0;
end;

procedure Tb2PrismaticJointDef.Initialize(bodyA, bodyB: Tb2Body; const anchor,
   axis: TVector2);
begin
   Self.bodyA := bodyA;
   Self.bodyB := bodyB;
   localAnchorA := bodyA.GetLocalPoint(anchor);
   localAnchorB := bodyB.GetLocalPoint(anchor);
   localAxis1 := bodyA.GetLocalVector(axis);
   referenceAngle := bodyB.GetAngle - bodyA.GetAngle;
end;

{ Tb2PrismaticJoint }

constructor Tb2PrismaticJoint.Create(def: Tb2PrismaticJointDef);
begin
   inherited Create(def);
   m_localAnchor1 := def.localAnchorA;
   m_localAnchor2 := def.localAnchorB;
   m_localXAxis1 := def.localAxis1;
   m_localYAxis1 := b2Cross(1.0, m_localXAxis1);
   m_refAngle := def.referenceAngle;

   m_impulse := b2Vec3_Zero;
   m_motorMass := 0.0;
   m_motorImpulse := 0.0;

   m_lowerTranslation := def.lowerTranslation;
   m_upperTranslation := def.upperTranslation;
   m_maxMotorForce := def.maxMotorForce;
   m_motorSpeed := def.motorSpeed;
   m_enableLimit := def.enableLimit;
   m_enableMotor := def.enableMotor;
   m_limitState := e_inactiveLimit;

   m_axis := b2Vec2_Zero;
   m_perp := b2Vec2_Zero;
end;

function Tb2PrismaticJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchor1);
end;

function Tb2PrismaticJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchor2);
end;

function Tb2PrismaticJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
   {$ELSE}
   Result := Multiply(Add(Multiply(m_perp, m_impulse.x), Multiply(m_axis,
      m_motorImpulse + m_impulse.z)), inv_dt);
   {$ENDIF}
end;

function Tb2PrismaticJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := inv_dt * m_impulse.y;
end;

procedure Tb2PrismaticJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
  r1, r2, d, P: TVector2;
  m1, m2, i1, i2, k11, k12, k13, k22, k23, k33, jointTranslation, L1, L2: Float;
begin
   m_localCenterA := m_bodyA.GetLocalCenter;
   m_localCenterB := m_bodyB.GetLocalCenter;

   // Compute the effective masses.
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_localCenterA);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_localCenterB);
   d := m_bodyB.m_sweep.c + r2 - m_bodyA.m_sweep.c - r1;
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_localCenterA));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_localCenterB));
   d := Subtract(Add(m_bodyB.m_sweep.c, r2), Add(m_bodyA.m_sweep.c, r1));
   {$ENDIF}

   m_invMassA := m_bodyA.m_invMass;
   m_invIA := m_bodyA.m_invI;
   m_invMassB := m_bodyB.m_invMass;
   m_invIB := m_bodyB.m_invI;

   // Compute motor Jacobian and effective mass.
   begin
      m_axis := b2Mul(m_bodyA.m_xf.R, m_localXAxis1);
      {$IFDEF OP_OVERLOAD}
      m_a1 := b2Cross(d + r1, m_axis);
      {$ELSE}
      m_a1 := b2Cross(Add(d, r1), m_axis);
      {$ENDIF}
      m_a2 := b2Cross(r2, m_axis);

      m_motorMass := m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;
      if m_motorMass > FLT_EPSILON then
         m_motorMass := 1.0 / m_motorMass;
   end;

   // Prismatic constraint.
   begin
      m_perp := b2Mul(m_bodyA.m_xf.R, m_localYAxis1);

      {$IFDEF OP_OVERLOAD}
      m_s1 := b2Cross(d + r1, m_perp);
      {$ELSE}
      m_s1 := b2Cross(Add(d, r1), m_perp);
      {$ENDIF}
      m_s2 := b2Cross(r2, m_perp);

      m1 := m_invMassA;
      m2 := m_invMassB;
      i1 := m_invIA;
      i2 := m_invIB;

      k11 := m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
      k12 := i1 * m_s1 + i2 * m_s2;
      k13 := i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
      k22 := i1 + i2;
      k23 := i1 * m_a1 + i2 * m_a2;
      k33 := m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

      {$IFDEF OP_OVERLOAD}
      m_K.col1.SetValue(k11, k12, k13);
      m_K.col2.SetValue(k12, k22, k23);
      m_K.col3.SetValue(k13, k23, k33);
      {$ELSE}
      SetValue(m_K.col1, k11, k12, k13);
      SetValue(m_K.col2, k12, k22, k23);
      SetValue(m_K.col3, k13, k23, k33);
      {$ENDIF}
   end;

   // Compute motor and limit terms.
   if m_enableLimit then
   begin
      jointTranslation := b2Dot(m_axis, d);
      if Abs(m_upperTranslation - m_lowerTranslation) < 2.0 * b2_linearSlop then
         m_limitState := e_equalLimits
      else if jointTranslation <= m_lowerTranslation then
      begin
         if m_limitState <> e_atLowerLimit then
         begin
            m_limitState := e_atLowerLimit;
            m_impulse.z := 0.0;
         end;
      end
      else if jointTranslation >= m_upperTranslation then
      begin
         if m_limitState <> e_atUpperLimit then
         begin
            m_limitState := e_atUpperLimit;
            m_impulse.z := 0.0;
         end;
      end
      else
      begin
         m_limitState := e_inactiveLimit;
         m_impulse.z := 0.0;
      end;
   end
   else
   begin
      m_limitState := e_inactiveLimit;
      m_impulse.z := 0.0;
   end;

   if not m_enableMotor then
     m_motorImpulse := 0.0;

   if step.warmStarting then
   begin
      // Account for variable time step.
      {$IFDEF OP_OVERLOAD}
      m_impulse.MultiplyBy(step.dtRatio);
      {$ELSE}
      MultiplyBy(m_impulse, step.dtRatio);
      {$ENDIF}
      m_motorImpulse := m_motorImpulse * step.dtRatio;

      L1 := m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
      L2 := m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;
      {$IFDEF OP_OVERLOAD}
      P := m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
      m_bodyA.m_linearVelocity.SubtractBy(m_invMassA * P);
      m_bodyB.m_linearVelocity.AddBy(m_invMassB * P);
      {$ELSE}
      P := Add(Multiply(m_perp, m_impulse.x), Multiply(m_axis, m_motorImpulse + m_impulse.z));
      SubtractBy(m_bodyA.m_linearVelocity, Multiply(P, m_invMassA));
      AddBy(m_bodyB.m_linearVelocity, Multiply(P, m_invMassB));
      {$ENDIF}

      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity - m_invIA * L1;
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_invIB * L2;
   end
   else
   begin
      m_impulse := b2Vec3_Zero;
      m_motorImpulse := 0.0;
   end;
end;

procedure Tb2PrismaticJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   v1, v2, P, Cdot1, b, f2r, df: TVector2;
   w1, w2, fCdot, Cdot2, impulse, oldImpulse, maxImpulse, L1, L2: Float;
   Cdot, f1, df3: TVector3;
begin
   v1 := m_bodyA.m_linearVelocity;
   w1 := m_bodyA.m_angularVelocity;
   v2 := m_bodyB.m_linearVelocity;
   w2 := m_bodyB.m_angularVelocity;

   // Solve linear motor constraint.
   if m_enableMotor and (m_limitState <> e_equalLimits) then
   begin
      {$IFDEF OP_OVERLOAD}
      fCdot := b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
      {$ELSE}
      fCdot := b2Dot(m_axis, Subtract(v2, v1)) + m_a2 * w2 - m_a1 * w1;
      {$ENDIF}
      impulse := m_motorMass * (m_motorSpeed - fCdot);
      oldImpulse := m_motorImpulse;
      maxImpulse := step.dt * m_maxMotorForce;
      m_motorImpulse := b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse := m_motorImpulse - oldImpulse;

      {$IFDEF OP_OVERLOAD}
      P := impulse * m_axis;
      {$ELSE}
      P := Multiply(m_axis, impulse);
      {$ENDIF}
      L1 := impulse * m_a1;
      L2 := impulse * m_a2;

      {$IFDEF OP_OVERLOAD}
      v1.SubtractBy(m_invMassA * P);
      v2.AddBy(m_invMassB * P);
      {$ELSE}
      SubtractBy(v1, Multiply(P, m_invMassA));
      AddBy(v2, Multiply(P, m_invMassB));
      {$ENDIF}
      w1 := w1 - m_invIA * L1;
      w2 := w2 + m_invIB * L2;
   end;

   {$IFDEF OP_OVERLOAD}
   Cdot1.x := b2Dot(m_perp, v2 - v1) + m_s2 * w2 - m_s1 * w1;
   {$ELSE}
   Cdot1.x := b2Dot(m_perp, Subtract(v2, v1)) + m_s2 * w2 - m_s1 * w1;
   {$ENDIF}
   Cdot1.y := w2 - w1;

   if m_enableLimit and (m_limitState <> e_inactiveLimit) then
   begin
      // Solve prismatic and limit constraint in block form.
      f1 := m_impulse;
      {$IFDEF OP_OVERLOAD}
      Cdot2 := b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
      Cdot.SetValue(Cdot1.x, Cdot1.y, Cdot2);
      df3 :=  m_K.Solve33(-Cdot);
      m_impulse.AddBy(df3);
      {$ELSE}
      Cdot2 := b2Dot(m_axis, Subtract(v2, v1)) + m_a2 * w2 - m_a1 * w1;
      SetValue(Cdot, Cdot1.x, Cdot1.y, Cdot2);
      df3 :=  Solve33(m_K, Negative(Cdot));
      AddBy(m_impulse, df3);
      {$ENDIF}

      if m_limitState = e_atLowerLimit then
         m_impulse.z := b2Max(m_impulse.z, 0.0)
      else if m_limitState = e_atUpperLimit then
         m_impulse.z := b2Min(m_impulse.z, 0.0);

      // f2(1:2) := invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
      {$IFDEF OP_OVERLOAD}
      b := -Cdot1 - (m_impulse.z - f1.z) * MakeVector(m_K.col3.x, m_K.col3.y);
      f2r := m_K.Solve22(b) + MakeVector(f1.x, f1.y);
      m_impulse.x := f2r.x;
      m_impulse.y := f2r.y;

      df3 := m_impulse - f1;

      P := df3.x * m_perp + df3.z * m_axis;
      L1 := df3.x * m_s1 + df3.y + df3.z * m_a1;
      L2 := df3.x * m_s2 + df3.y + df3.z * m_a2;

      v1.SubtractBy(m_invMassA * P);
      v2.AddBy(m_invMassB * P);
      {$ELSE}
      b := Negative(Add(Cdot1, Multiply(MakeVector(m_K.col3.x, m_K.col3.y), m_impulse.z - f1.z)));
      f2r := Add(Solve22(m_K, b), MakeVector(f1.x, f1.y));
      m_impulse.x := f2r.x;
      m_impulse.y := f2r.y;

      df3 := Subtract(m_impulse, f1);

      P := Add(Multiply(m_perp, df3.x), Multiply(m_axis, df3.z));
      L1 := df3.x * m_s1 + df3.y + df3.z * m_a1;
      L2 := df3.x * m_s2 + df3.y + df3.z * m_a2;

      SubtractBy(v1, Multiply(P, m_invMassA));
      AddBy(v2, Multiply(P, m_invMassB));
      {$ENDIF}

      w1 := w1 - m_invIA * L1;
      w2 := w2 + m_invIB * L2;
   end
   else
   begin
      // Limit is inactive, just solve the prismatic constraint in block form.
      {$IFDEF OP_OVERLOAD}
      df := m_K.Solve22(-Cdot1);
      m_impulse.x := m_impulse.x + df.x;
      m_impulse.y := m_impulse.y + df.y;

      P := df.x * m_perp;
      L1 := df.x * m_s1 + df.y;
      L2 := df.x * m_s2 + df.y;

      v1.SubtractBy(m_invMassA * P);
      v2.AddBy(m_invMassB * P);
      {$ELSE}
      df := Solve22(m_K, Negative(Cdot1));
      m_impulse.x := m_impulse.x + df.x;
      m_impulse.y := m_impulse.y + df.y;

      P := Multiply(m_perp, df.x);
      L1 := df.x * m_s1 + df.y;
      L2 := df.x * m_s2 + df.y;

      SubtractBy(v1, Multiply(P, m_invMassA));
      AddBy(v2, Multiply(P, m_invMassB));
      {$ENDIF}
      w1 := w1 - m_invIA * L1;
      w2 := w2 + m_invIB * L2;
   end;

   m_bodyA.m_linearVelocity := v1;
   m_bodyA.m_angularVelocity := w1;
   m_bodyB.m_linearVelocity := v2;
   m_bodyB.m_angularVelocity := w2;
end;

function Tb2PrismaticJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
var
   r1, r2, d, c1, c2, _C1, impulse1, P: TVector2;
   a1, a2, linearError, angularError, _C2, translation,
   m1, i1, m2, i2, k11, k12, k13, k22, k23, k33, L1, L2: Float;
   active: Boolean;
   _R1, _R2: TMatrix22;
   impulse, C: TVector3;
begin
   //B2_NOT_USED(baumgarte);
   c1 := m_bodyA.m_sweep.c;
   a1 := m_bodyA.m_sweep.a;
   c2 := m_bodyB.m_sweep.c;
   a2 := m_bodyB.m_sweep.a;

   // Solve linear limit constraint.
   linearError := 0.0;
   //angularError := 0.0;
   active := False;
   _C2 := 0.0;

   {$IFDEF OP_OVERLOAD}
   _R1.SetValue(a1);
   _R2.SetValue(a2);
   r1 := b2Mul(_R1, m_localAnchor1 - m_localCenterA);
   r2 := b2Mul(_R2, m_localAnchor2 - m_localCenterB);
   d := c2 + r2 - c1 - r1;
   {$ELSE}
   SetValue(_R1, a1);
   SetValue(_R2, a2);
   r1 := b2Mul(_R1, Subtract(m_localAnchor1, m_localCenterA));
   r2 := b2Mul(_R2, Subtract(m_localAnchor2, m_localCenterB));
   d := Subtract(Add(c2, r2), Add(c1, r1));
   {$ENDIF}

   if m_enableLimit then
   begin
      m_axis := b2Mul(_R1, m_localXAxis1);

      {$IFDEF OP_OVERLOAD}
      m_a1 := b2Cross(d + r1, m_axis);
      {$ELSE}
      m_a1 := b2Cross(Add(d, r1), m_axis);
      {$ENDIF}
      m_a2 := b2Cross(r2, m_axis);

      translation := b2Dot(m_axis, d);
      if Abs(m_upperTranslation - m_lowerTranslation) < 2.0 * b2_linearSlop then
      begin
         // Prevent large angular corrections
         _C2 := b2Clamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
         linearError := Abs(translation);
         active := True;
      end
      else if translation <= m_lowerTranslation then
      begin
         // Prevent large linear corrections and allow some slop.
         _C2 := b2Clamp(translation - m_lowerTranslation + b2_linearSlop, -b2_maxLinearCorrection, 0.0);
         linearError := m_lowerTranslation - translation;
         active := True;
      end
      else if translation >= m_upperTranslation then
      begin
         // Prevent large linear corrections and allow some slop.
         _C2 := b2Clamp(translation - m_upperTranslation - b2_linearSlop, 0.0, b2_maxLinearCorrection);
         linearError := translation - m_upperTranslation;
         active := True;
      end;
   end;

   m_perp := b2Mul(_R1, m_localYAxis1);

   {$IFDEF OP_OVERLOAD}
   m_s1 := b2Cross(d + r1, m_perp);
   {$ELSE}
   m_s1 := b2Cross(Add(d, r1), m_perp);
   {$ENDIF}
   m_s2 := b2Cross(r2, m_perp);

   _C1.x := b2Dot(m_perp, d);
   _C1.y := a2 - a1 - m_refAngle;

   linearError := b2Max(linearError, Abs(_C1.x));
   angularError := Abs(_C1.y);

   m1 := m_invMassA;
   m2 := m_invMassB;
   i1 := m_invIA;
   i2 := m_invIB;
   if active then
   begin
      k11 := m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
      k12 := i1 * m_s1 + i2 * m_s2;
      k13 := i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
      k22 := i1 + i2;
      k23 := i1 * m_a1 + i2 * m_a2;
      k33 := m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

      {$IFDEF OP_OVERLOAD}
      m_K.col1.SetValue(k11, k12, k13);
      m_K.col2.SetValue(k12, k22, k23);
      m_K.col3.SetValue(k13, k23, k33);
      {$ELSE}
      SetValue(m_K.col1, k11, k12, k13);
      SetValue(m_K.col2, k12, k22, k23);
      SetValue(m_K.col3, k13, k23, k33);
      {$ENDIF}

      C.x := _C1.x;
      C.y := _C1.y;
      C.z := _C2;

      {$IFDEF OP_OVERLOAD}
      impulse := m_K.Solve33(-C);
      {$ELSE}
      impulse := Solve33(m_K, Negative(C));
      {$ENDIF}
   end
   else
   begin
      k11 := m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
      k12 := i1 * m_s1 + i2 * m_s2;
      k22 := i1 + i2;

      {$IFDEF OP_OVERLOAD}
      m_K.col1.SetValue(k11, k12, 0.0);
      m_K.col2.SetValue(k12, k22, 0.0);

      impulse1 := m_K.Solve22(-_C1);
      {$ELSE}
      SetValue(m_K.col1, k11, k12, 0.0);
      SetValue(m_K.col2, k12, k22, 0.0);

      impulse1 := Solve22(m_K, Negative(_C1));
      {$ENDIF}

      impulse.x := impulse1.x;
      impulse.y := impulse1.y;
      impulse.z := 0.0;
   end;

   {$IFDEF OP_OVERLOAD}
   P := impulse.x * m_perp + impulse.z * m_axis;
   {$ELSE}
   P := Add(Multiply(m_perp, impulse.x), Multiply(m_axis, impulse.z));
   {$ENDIF}
   L1 := impulse.x * m_s1 + impulse.y + impulse.z * m_a1;
   L2 := impulse.x * m_s2 + impulse.y + impulse.z * m_a2;

   {$IFDEF OP_OVERLOAD}
   c1.SubtractBy(m_invMassA * P);
   c2.AddBy(m_invMassB * P);
   {$ELSE}
   SubtractBy(c1, Multiply(P, m_invMassA));
   AddBy(c2, Multiply(P, m_invMassB));
   {$ENDIF}

   a1 := a1 - m_invIA * L1;
   a2 := a2 + m_invIB * L2;

   // TODO_ERIN remove need for this.
   m_bodyA.m_sweep.c := c1;
   m_bodyA.m_sweep.a := a1;
   m_bodyB.m_sweep.c := c2;
   m_bodyB.m_sweep.a := a2;
   m_bodyA.SynchronizeTransform;
 	 m_bodyB.SynchronizeTransform;

	 Result := (linearError <= b2_linearSlop) and (angularError <= b2_angularSlop);
end;

function Tb2PrismaticJoint.GetJointTranslation: Float;
var
   d, axis: TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   d := m_bodyB.GetWorldPoint(m_localAnchor2) - m_bodyA.GetWorldPoint(m_localAnchor1);
   {$ELSE}
   d := Subtract(m_bodyB.GetWorldPoint(m_localAnchor2), m_bodyA.GetWorldPoint(m_localAnchor1));
   {$ENDIF}
   axis := m_bodyA.GetWorldVector(m_localXAxis1);
   Result := b2Dot(d, axis);
end;

procedure Tb2PrismaticJoint.EnableLimit(flag: Boolean);
begin
	 m_bodyA.SetAwake(True);
	 m_bodyB.SetAwake(True);
	 m_enableLimit := flag;
end;

procedure Tb2PrismaticJoint.EnableMotor(flag: Boolean);
begin
	 m_bodyA.SetAwake(True);
	 m_bodyB.SetAwake(True);
	 m_enableMotor := flag;
end;

function Tb2PrismaticJoint.GetJointSpeed: Float;
var
   r1, r2, d, axis: TVector2;
   w1: Float;
begin
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   d := (m_bodyB.m_sweep.c + r2) - (m_bodyA.m_sweep.c + r1);
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   d := Subtract(Add(m_bodyB.m_sweep.c, r2), Add(m_bodyA.m_sweep.c, r1));
   {$ENDIF}
   axis := m_bodyA.GetWorldVector(m_localXAxis1);

   w1 := m_bodyA.m_angularVelocity;

   {$IFDEF OP_OVERLOAD}
   Result := b2Dot(d, b2Cross(w1, axis)) +
      b2Dot(axis, m_bodyB.m_linearVelocity + b2Cross(m_bodyB.m_angularVelocity, r2) -
      m_bodyA.m_linearVelocity - b2Cross(w1, r1));
   {$ELSE}
   Result := b2Dot(d, b2Cross(w1, axis)) +
      b2Dot(axis, Subtract(Add(m_bodyB.m_linearVelocity, b2Cross(m_bodyB.m_angularVelocity, r2)),
      Add(m_bodyA.m_linearVelocity, b2Cross(w1, r1))));
   {$ENDIF}
end;

procedure Tb2PrismaticJoint.SetLimits(lower, upper: Float);
begin
   //b2Assert(lower <= upper);
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
	 m_lowerTranslation := lower;
	 m_upperTranslation := upper;
end;

procedure Tb2PrismaticJoint.SetMotorSpeed(speed: Float);
begin
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
	 m_motorSpeed := speed;
end;

procedure Tb2PrismaticJoint.SetMaxMotorForce(force: Float);
begin
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
   m_maxMotorForce := force;
end;

{ Tb2MouseJointDef }

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

constructor Tb2MouseJointDef.Create;
begin
    inherited;
		JointType := e_mouseJoint;
    target := b2Vec2_Zero;
		maxForce := 0.0;
		frequencyHz := 5.0;
		dampingRatio := 0.7;
end;

{ Tb2MouseJoint }

constructor Tb2MouseJoint.Create(def: Tb2MouseJointDef);
begin
   //b2Assert(def->target.IsValid());
   //b2Assert(b2IsValid(def->maxForce) && def->maxForce >= 0.0f);
   //b2Assert(b2IsValid(def->frequencyHz) && def->frequencyHz >= 0.0f);
   //b2Assert(b2IsValid(def->dampingRatio) && def->dampingRatio >= 0.0f);

   inherited Create(def);
   m_target := def.target;
   m_localAnchor := b2MulT(m_bodyB.m_xf, m_target);

   m_maxForce := def.maxForce;
   m_impulse := b2Vec2_Zero;

   m_frequencyHz := def.frequencyHz;
   m_dampingRatio := def.dampingRatio;

   m_beta := 0.0;
   m_gamma := 0.0;
end;

function Tb2MouseJoint.GetAnchorA: TVector2;
begin
   Result := m_target;
end;

function Tb2MouseJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchor);
end;

function Tb2MouseJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := inv_dt * m_impulse;
   {$ELSE}
   Result := Multiply(m_impulse, inv_dt);
   {$ENDIF}
end;

function Tb2MouseJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := 0.0;
end;

procedure Tb2MouseJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   r: TVector2;
   invMass, invI, omega, d, _k: Float;
   K1, K2, K: TMatrix22;
begin
   // Frequency
   omega := 2.0 * Pi * m_frequencyHz;

   // Damping coefficient
   d := 2.0 * m_bodyB.m_mass * m_dampingRatio * omega;

   // Spring stiffness
   _k := m_bodyB.m_mass * (omega * omega);

   // magic formulas
   // gamma has units of inverse mass.
   // beta has units of inverse time.
   //b2Assert(d + step.dt * k > b2_epsilon);
   m_gamma := step.dt * (d + step.dt * _k);
   if m_gamma <> 0.0 then
      m_gamma := 1.0 / m_gamma;
   m_beta := step.dt * _k * m_gamma;

   // Compute the effective mass matrix.
   {$IFDEF OP_OVERLOAD}
   r := b2Mul(m_bodyB.m_xf.R, m_localAnchor - m_bodyB.GetLocalCenter);
   {$ELSE}
   r := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor, m_bodyB.GetLocalCenter));
   {$ENDIF}

   // K    := [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
   //      := [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
   //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
   invMass := m_bodyB.m_invMass;
   invI := m_bodyB.m_invI;

   K1.col1.x := invMass;
   K1.col2.x := 0.0;
   K1.col1.y := 0.0;
   K1.col2.y := invMass;

   K2.col1.x :=  invI * r.y * r.y;
   K2.col2.x := -invI * r.x * r.y;
   K2.col1.y := K2.col2.x;
   K2.col2.y :=  invI * r.x * r.x;

   {$IFDEF OP_OVERLOAD}
   K := K1 + K2;
   {$ELSE}
   K := Add(K1, K2);
   {$ENDIF}
   K.col1.x := K.col1.x + m_gamma;
   K.col2.y := K.col2.y + m_gamma;

   {$IFDEF OP_OVERLOAD}
   m_mass := K.Invert;
   m_C := m_bodyB.m_sweep.c + r - m_target;
   {$ELSE}
   m_mass := Invert(K);
   m_C := Add(m_bodyB.m_sweep.c, r);
   SubtractBy(m_C, m_target);
   {$ENDIF}
   m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity * 0.98; // Cheat with some damping

   // Warm starting.
   {$IFDEF OP_OVERLOAD}
   m_impulse.MultiplyBy(step.dtRatio);
   m_bodyB.m_linearVelocity.AddBy(invMass * m_impulse);
   {$ELSE}
   MultiplyBy(m_impulse, step.dtRatio);
   AddBy(m_bodyB.m_linearVelocity, Multiply(m_impulse, invMass));
   {$ENDIF}
   m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + invI * b2Cross(r, m_impulse);
end;

procedure Tb2MouseJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   r, Cdot, impulse, oldImpulse: TVector2;
   maxImpulse: Float;
begin
   oldImpulse := m_impulse;
   // Cdot := v + cross(w, r)
   {$IFDEF OP_OVERLOAD}
   r := b2Mul(m_bodyB.m_xf.R, m_localAnchor - m_bodyB.GetLocalCenter);
   Cdot := m_bodyB.m_linearVelocity + b2Cross(m_bodyB.m_angularVelocity, r);
   impulse := b2Mul(m_mass, -(Cdot + m_beta * m_C + m_gamma * m_impulse));
   m_impulse.AddBy(impulse);
   maxImpulse := step.dt * m_maxForce;
   if m_impulse.SqrLength > maxImpulse * maxImpulse then
      m_impulse.MultiplyBy(maxImpulse / m_impulse.Length);
   impulse := m_impulse - oldImpulse;

 	 m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * impulse);
   {$ELSE}
   r := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor, m_bodyB.GetLocalCenter));
   Cdot := Add(m_bodyB.m_linearVelocity, b2Cross(m_bodyB.m_angularVelocity, r));
   impulse := b2Mul(m_mass, Negative(Add(Cdot, Multiply(m_C, m_beta), Multiply(m_impulse, m_gamma))));
   AddBy(m_impulse, impulse);
   maxImpulse := step.dt * m_maxForce;
   if SqrLength(m_impulse) > maxImpulse * maxImpulse then
      MultiplyBy(m_impulse, maxImpulse / LengthVec(m_impulse));
   impulse := Subtract(m_impulse, oldImpulse);

   AddBy(m_bodyB.m_linearVelocity, Multiply(impulse, m_bodyB.m_invMass));
   {$ENDIF}

   m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity +
      m_bodyB.m_invI * b2Cross(r, impulse);
end;

function Tb2MouseJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
begin
   Result := True;
end;

procedure Tb2MouseJoint.SetTarget(const target: TVector2);
begin
   if not m_bodyB.IsAwake then
      m_bodyB.SetAwake(True);
   m_target := target;
end;

function Tb2MouseJoint.GetTarget: TVector2;
begin
   Result := m_target;
end;

{ Tb2PulleyJointDef }

// Pulley:
// lengthA = norm(p1 - s1)
// lengthB = norm(p2 - s2)
// C0 = (lengthA + ratio * lengthB)_initial
// C = C0 - (lengthA + ratio * lengthB) >= 0
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
//
// Limit:
// C = maxLength - length
// u = (p - s) / norm(p - s)
// Cdot = -dot(u, v + cross(w, r))
// K = invMass + invI * cross(r, u)^2
// 0 <= impulse

const
   b2_minPulleyLength = 2.0;

constructor Tb2PulleyJointDef.Create;
begin
   inherited;
   JointType := e_pulleyJoint;
   SetValue(groundAnchorA, -1.0, 1.0);
   SetValue(groundAnchorB, 1.0, 1.0);
   SetValue(localAnchorA, -1.0, 0.0);
   SetValue(localAnchorB, 1.0, 0.0);
   lengthA := 0.0;
   maxLengthA := 0.0;
   lengthB := 0.0;
   maxLengthB := 0.0;
   ratio := 1.0;
   collideConnected := True;
end;

procedure Tb2PulleyJointDef.Initialize(bodyA, bodyB: Tb2Body; const groundAnchorA,
   groundAnchorB, anchorA, anchorB: TVector2; ratio: Float);
var
   C: Float;
begin
   Self.bodyA := bodyA;
   Self.bodyB := bodyB;
   Self.groundAnchorA := groundAnchorA;
   Self.groundAnchorB := groundAnchorB;
   Self.localAnchorA := bodyA.GetLocalPoint(anchorA);
   Self.localAnchorB := bodyB.GetLocalPoint(anchorB);
   {$IFDEF OP_OVERLOAD}
   lengthA := (anchorA - groundAnchorA).Length;
   lengthB := (anchorB - groundAnchorB).Length;
   {$ELSE}
   lengthA := LengthVec(Subtract(anchorA, groundAnchorA));
   lengthB := LengthVec(Subtract(anchorB, groundAnchorB));
   {$ENDIF}
   Self.ratio := ratio;
   //b2Assert(ratio > B2_FLT_EPSILON);
   C := lengthA + ratio * lengthB;
   maxLengthA := C - ratio * b2_minPulleyLength;
   maxLengthB := (C - b2_minPulleyLength) / ratio;
end;

{ Tb2PulleyJoint }

constructor Tb2PulleyJoint.Create(def: Tb2PulleyJointDef);
begin
   inherited Create(def);
   m_groundAnchor1 := def.groundAnchorA;
   m_groundAnchor2 := def.groundAnchorB;
   m_localAnchor1 := def.localAnchorA;
   m_localAnchor2 := def.localAnchorB;

   //b2Assert(def.ratio != 0.0);
   m_ratio := def.ratio;
   m_constant := def.lengthA + m_ratio * def.lengthB;

   m_maxLength1 := b2Min(def.maxLengthA, m_constant - m_ratio * b2_minPulleyLength);
   m_maxLength2 := b2Min(def.maxLengthB, (m_constant - b2_minPulleyLength) / m_ratio);

   m_impulse := 0.0;
   m_limitImpulse1 := 0.0;
   m_limitImpulse2 := 0.0;
end;

function Tb2PulleyJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchor1);
end;

function Tb2PulleyJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchor2);
end;

function Tb2PulleyJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := (m_impulse * inv_dt) * m_u2;
   {$ELSE}
   Result := Multiply(m_u2, m_impulse * inv_dt);
   {$ENDIF}
end;

function Tb2PulleyJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := 0.0;
end;

procedure Tb2PulleyJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   r1, r2, P1, P2: TVector2;
   lengthA, lengthB, C, cr1u1, cr2u2: Float;
begin
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);

   // Get the pulley axes.
   m_u1 := m_bodyA.m_sweep.c + r1 - m_groundAnchor1;
   m_u2 := m_bodyB.m_sweep.c + r2 - m_groundAnchor2;

   lengthA := m_u1.Length;
   lengthB := m_u2.Length;

   if lengthA > b2_linearSlop then
      m_u1.DivideBy(lengthA)
   else
      m_u1 := b2Vec2_Zero;

   if lengthB > b2_linearSlop then
      m_u2.DivideBy(lengthB)
   else
      m_u2 := b2Vec2_Zero;
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));

   // Get the pulley axes.
   m_u1 := Subtract(Add(m_bodyA.m_sweep.c, r1), m_groundAnchor1);
   m_u2 := Subtract(Add(m_bodyB.m_sweep.c, r2), m_groundAnchor2);

   lengthA := LengthVec(m_u1);
   lengthB := LengthVec(m_u2);

   if lengthA > b2_linearSlop then
      DivideBy(m_u1, lengthA)
   else
      m_u1 := b2Vec2_Zero;

   if lengthB > b2_linearSlop then
      DivideBy(m_u2, lengthB)
   else
      m_u2 := b2Vec2_Zero;
   {$ENDIF}

   C := m_constant - lengthA - m_ratio * lengthB;
   if C > 0.0 then
   begin
      m_state := e_inactiveLimit;
      m_impulse := 0.0;
   end
   else
      m_state := e_atUpperLimit;

   if lengthA < m_maxLength1 then
   begin
      m_limitState1 := e_inactiveLimit;
      m_limitImpulse1 := 0.0;
   end
   else
      m_limitState1 := e_atUpperLimit;

   if lengthB < m_maxLength2 then
   begin
      m_limitState2 := e_inactiveLimit;
      m_limitImpulse2 := 0.0;
   end
   else
      m_limitState2 := e_atUpperLimit;

   // Compute effective mass.
   cr1u1 := b2Cross(r1, m_u1);
   cr2u2 := b2Cross(r2, m_u2);

   m_limitMass1 := m_bodyA.m_invMass + m_bodyA.m_invI * cr1u1 * cr1u1;
   m_limitMass2 := m_bodyB.m_invMass + m_bodyB.m_invI * cr2u2 * cr2u2;
   m_pulleyMass := m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
   //b2Assert(m_limitMass1 > B2_FLT_EPSILON);
   //b2Assert(m_limitMass2 > B2_FLT_EPSILON);
   //b2Assert(m_pulleyMass > B2_FLT_EPSILON);
   m_limitMass1 := 1.0 / m_limitMass1;
   m_limitMass2 := 1.0 / m_limitMass2;
   m_pulleyMass := 1.0 / m_pulleyMass;

   if step.warmStarting then
   begin
      // Scale impulses to support variable time steps.
      m_impulse := m_impulse * step.dtRatio;
      m_limitImpulse1 := m_limitImpulse1 * step.dtRatio;
      m_limitImpulse2 := m_limitImpulse2 * step.dtRatio;

      // Warm starting.
      {$IFDEF OP_OVERLOAD}
      P1 := -(m_impulse + m_limitImpulse1) * m_u1;
      P2 := (-m_ratio * m_impulse - m_limitImpulse2) * m_u2;
      m_bodyA.m_linearVelocity.AddBy(m_bodyA.m_invMass * P1);
      m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * P2);
      {$ELSE}
      P1 := Multiply(m_u1, -(m_impulse + m_limitImpulse1));
      P2 := Multiply(m_u2, (-m_ratio * m_impulse - m_limitImpulse2));
      AddBy(m_bodyA.m_linearVelocity, Multiply(P1, m_bodyA.m_invMass));
      AddBy(m_bodyB.m_linearVelocity, Multiply(P2, m_bodyB.m_invMass));
      {$ENDIF}
      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity + m_bodyA.m_invI * b2Cross(r1, P1);
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * b2Cross(r2, P2);
   end
   else
   begin
		  m_impulse := 0.0;
		  m_limitImpulse1 := 0.0;
		  m_limitImpulse2 := 0.0;
   end;
end;

procedure Tb2PulleyJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   r1, r2, v1, v2, P1, P2: TVector2;
   Cdot, impulse, oldImpulse: Float;
begin
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   {$ENDIF}

   if m_state = e_atUpperLimit then
   begin
      {$IFDEF OP_OVERLOAD}
      v1 := m_bodyA.m_linearVelocity + b2Cross(m_bodyA.m_angularVelocity, r1);
      v2 := m_bodyB.m_linearVelocity + b2Cross(m_bodyB.m_angularVelocity, r2);
      {$ELSE}
      v1 := Add(m_bodyA.m_linearVelocity, b2Cross(m_bodyA.m_angularVelocity, r1));
      v2 := Add(m_bodyB.m_linearVelocity, b2Cross(m_bodyB.m_angularVelocity, r2));
      {$ENDIF}

      Cdot := -b2Dot(m_u1, v1) - m_ratio * b2Dot(m_u2, v2);
      impulse := m_pulleyMass * (-Cdot);
      oldImpulse := m_impulse;
      m_impulse := b2Max(0.0, m_impulse + impulse);
      impulse := m_impulse - oldImpulse;

      {$IFDEF OP_OVERLOAD}
      P1 := (-impulse) * m_u1;
		  P2 := -m_ratio * impulse * m_u2;
      m_bodyA.m_linearVelocity.AddBy(m_bodyA.m_invMass * P1);
      m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * P2);
      {$ELSE}
      P1 := Multiply(m_u1, -impulse);
		  P2 := Multiply(m_u2, -m_ratio * impulse);
      AddBy(m_bodyA.m_linearVelocity, Multiply(P1, m_bodyA.m_invMass));
      AddBy(m_bodyB.m_linearVelocity, Multiply(P2, m_bodyB.m_invMass));
      {$ENDIF}
      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity + m_bodyA.m_invI * b2Cross(r1, P1);
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * b2Cross(r2, P2);
   end;

   if m_limitState1 = e_atUpperLimit then
   begin
      {$IFDEF OP_OVERLOAD}
      v1 := m_bodyA.m_linearVelocity + b2Cross(m_bodyA.m_angularVelocity, r1);
      {$ELSE}
      v1 := Add(m_bodyA.m_linearVelocity, b2Cross(m_bodyA.m_angularVelocity, r1));
      {$ENDIF}

      Cdot := -b2Dot(m_u1, v1);
      impulse := -m_limitMass1 * Cdot;
      oldImpulse := m_limitImpulse1;
      m_limitImpulse1 := b2Max(0.0, m_limitImpulse1 + impulse);
      impulse := m_limitImpulse1 - oldImpulse;

      {$IFDEF OP_OVERLOAD}
      P1 := (-impulse) * m_u1;
      m_bodyA.m_linearVelocity.AddBy(m_bodyA.m_invMass * P1);
      {$ELSE}
      P1 := Multiply(m_u1, -impulse);
      AddBy(m_bodyA.m_linearVelocity, Multiply(P1, m_bodyA.m_invMass));
      {$ENDIF}
      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity + m_bodyA.m_invI * b2Cross(r1, P1);
   end;

   if m_limitState2 = e_atUpperLimit then
   begin
      {$IFDEF OP_OVERLOAD}
      v2 := m_bodyB.m_linearVelocity + b2Cross(m_bodyB.m_angularVelocity, r2);
      {$ELSE}
      v2 := Add(m_bodyB.m_linearVelocity, b2Cross(m_bodyB.m_angularVelocity, r2));
      {$ENDIF}

      Cdot := -b2Dot(m_u2, v2);
      impulse := -m_limitMass2 * Cdot;
      oldImpulse := m_limitImpulse2;
      m_limitImpulse2 := b2Max(0.0, m_limitImpulse2 + impulse);
      impulse := m_limitImpulse2 - oldImpulse;

      {$IFDEF OP_OVERLOAD}
      P2 := (-impulse) * m_u2;
      m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * P2);
      {$ELSE}
      P2 := Multiply(m_u2, -impulse);
      AddBy(m_bodyB.m_linearVelocity, Multiply(P2, m_bodyB.m_invMass));
      {$ENDIF}
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * b2Cross(r2, P2);
   end;
end;

function Tb2PulleyJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
var
   s1, s2, r1, r2, P1, P2: TVector2;
   linearError, lengthA, lengthB, C, impulse: Float;
begin
   s1 := m_groundAnchor1;
	 s2 := m_groundAnchor2;

   linearError := 0.0;

   if m_state = e_atUpperLimit then
   begin
      {$IFDEF OP_OVERLOAD}
      r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
      r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);

      // Get the pulley axes.
      m_u1 := m_bodyA.m_sweep.c + r1 - s1;
      m_u2 := m_bodyB.m_sweep.c + r2 - s2;

      lengthA := m_u1.Length;
      lengthB := m_u2.Length;

      if lengthA > b2_linearSlop then
         m_u1.DivideBy(lengthA)
      else
         m_u1 := b2Vec2_Zero;

      if lengthB > b2_linearSlop then
         m_u2.DivideBy(lengthB)
      else
         m_u2 := b2Vec2_Zero;
      {$ELSE}
      r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
      r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));

      // Get the pulley axes.
      m_u1 := Subtract(Add(m_bodyA.m_sweep.c, r1), s1);
      m_u2 := Subtract(Add(m_bodyB.m_sweep.c, r2), s2);

      lengthA := LengthVec(m_u1);
      lengthB := LengthVec(m_u2);

      if lengthA > b2_linearSlop then
         DivideBy(m_u1, lengthA)
      else
         m_u1 := b2Vec2_Zero;

      if lengthB > b2_linearSlop then
         DivideBy(m_u2, lengthB)
      else
         m_u2 := b2Vec2_Zero;
      {$ENDIF}

      C := m_constant - lengthA - m_ratio * lengthB;
      linearError := b2Max(linearError, -C);

      C := b2Clamp(C + b2_linearSlop, -b2_maxLinearCorrection, 0.0);
      impulse := -m_pulleyMass * C;

      {$IFDEF OP_OVERLOAD}
      P1 := -impulse * m_u1;
      P2 := -m_ratio * impulse * m_u2;

      m_bodyA.m_sweep.c.AddBy(m_bodyA.m_invMass * P1);
      m_bodyB.m_sweep.c.AddBy(m_bodyB.m_invMass * P2);
      {$ELSE}
      P1 := Multiply(m_u1, -impulse);
      P2 := Multiply(m_u2, -m_ratio * impulse);

      AddBy(m_bodyA.m_sweep.c, Multiply(P1, m_bodyA.m_invMass));
      AddBy(m_bodyB.m_sweep.c, Multiply(P2, m_bodyB.m_invMass));
      {$ENDIF}
      m_bodyA.m_sweep.a := m_bodyA.m_sweep.a + m_bodyA.m_invI * b2Cross(r1, P1);
      m_bodyB.m_sweep.a := m_bodyB.m_sweep.a + m_bodyB.m_invI * b2Cross(r2, P2);

      m_bodyA.SynchronizeTransform;
      m_bodyB.SynchronizeTransform;
   end;

   if m_limitState1 = e_atUpperLimit then
   begin
      {$IFDEF OP_OVERLOAD}
      r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
      p1 := m_bodyA.m_sweep.c + r1;

      m_u1 := p1 - s1;
      lengthA := m_u1.Length;

      if lengthA > b2_linearSlop then
         m_u1.DivideBy(lengthA)
      else
         m_u1 := b2Vec2_Zero;
      {$ELSE}
      r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
      p1 := Add(m_bodyA.m_sweep.c, r1);

      m_u1 := Subtract(p1, s1);
      lengthA := LengthVec(m_u1);

      if lengthA > b2_linearSlop then
         DivideBy(m_u1, lengthA)
      else
         m_u1 := b2Vec2_Zero;
      {$ENDIF}

      C := m_maxLength1 - lengthA;
      linearError := b2Max(linearError, -C);
      C := b2Clamp(C + b2_linearSlop, -b2_maxLinearCorrection, 0.0);
      impulse := -m_limitMass1 * C;

      {$IFDEF OP_OVERLOAD}
      P1 := -impulse * m_u1;
      m_bodyA.m_sweep.c.AddBy(m_bodyA.m_invMass * P1);
      {$ELSE}
      P1 := Multiply(m_u1, -impulse);
      AddBy(m_bodyA.m_sweep.c, Multiply(P1, m_bodyA.m_invMass));
      {$ENDIF}
      m_bodyA.m_sweep.a := m_bodyA.m_sweep.a + m_bodyA.m_invI * b2Cross(r1, P1);

      m_bodyA.SynchronizeTransform;
   end;

   if m_limitState2 = e_atUpperLimit then
   begin
      {$IFDEF OP_OVERLOAD}
      r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
      p2 := m_bodyB.m_sweep.c + r2;

      m_u2 := p2 - s2;
      lengthB := m_u2.Length;

      if lengthB > b2_linearSlop then
         m_u2.DivideBy(lengthB)
      else
         m_u2 := b2Vec2_Zero;
      {$ELSE}
      r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
      p2 := Add(m_bodyB.m_sweep.c, r2);

      m_u2 := Subtract(p2, s2);
      lengthB := LengthVec(m_u2);

      if lengthB > b2_linearSlop then
         DivideBy(m_u2, lengthB)
      else
         m_u2 := b2Vec2_Zero;
      {$ENDIF}

      C := m_maxLength2 - lengthB;
      linearError := b2Max(linearError, -C);
      C := b2Clamp(C + b2_linearSlop, -b2_maxLinearCorrection, 0.0);
      impulse := -m_limitMass2 * C;

      {$IFDEF OP_OVERLOAD}
      P2 := -impulse * m_u2;
      m_bodyB.m_sweep.c.AddBy(m_bodyB.m_invMass * P2);
      {$ELSE}
      P2 := Multiply(m_u2, -impulse);
      AddBy(m_bodyB.m_sweep.c, Multiply(P2, m_bodyB.m_invMass));
      {$ENDIF}
      m_bodyB.m_sweep.a := m_bodyB.m_sweep.a + m_bodyB.m_invI * b2Cross(r2, P2);

      m_bodyB.SynchronizeTransform;
   end;

   Result := linearError < b2_linearSlop;
end;

function Tb2PulleyJoint.GetLength1: Float;
begin
   {$IFDEF OP_OVERLOAD}
   Result := (m_bodyA.GetWorldPoint(m_localAnchor1) - m_groundAnchor1).Length;
   {$ELSE}
   Result := LengthVec(Subtract(m_bodyA.GetWorldPoint(m_localAnchor1), m_groundAnchor1));
   {$ENDIF}
end;

function Tb2PulleyJoint.GetLength2: Float;
begin
   {$IFDEF OP_OVERLOAD}
   Result := (m_bodyB.GetWorldPoint(m_localAnchor2) - m_groundAnchor2).Length;
   {$ELSE}
   Result := LengthVec(Subtract(m_bodyB.GetWorldPoint(m_localAnchor2), m_groundAnchor2));
   {$ENDIF}
end;

function Tb2PulleyJoint.GetGroundAnchorA: TVector2;
begin
   Result := m_groundAnchor1;
end;

function Tb2PulleyJoint.GetGroundAnchorB: TVector2;
begin
   Result := m_groundAnchor2;
end;

{ Tb2RevoluteJointDef }

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

constructor Tb2RevoluteJointDef.Create;
begin
   inherited;
   JointType := e_revoluteJoint;
   localAnchorA := b2Vec2_Zero;
   localAnchorB := b2Vec2_Zero;
   referenceAngle := 0.0;
   lowerAngle := 0.0;
   upperAngle := 0.0;
   maxMotorTorque := 0.0;
   motorSpeed := 0.0;
   enableLimit := False;
   enableMotor := False;
   motorOnBodyB := False;
end;

procedure Tb2RevoluteJointDef.Initialize(bodyA, bodyB: Tb2Body; const anchor: TVector2);
begin
   Self.bodyA := bodyA;
	 Self.bodyB := bodyB;
	 localAnchorA := bodyA.GetLocalPoint(anchor);
	 localAnchorB := bodyB.GetLocalPoint(anchor);
	 referenceAngle := bodyB.GetAngle - bodyA.GetAngle;
end;

{ Tb2RevoluteJoint }

constructor Tb2RevoluteJoint.Create(def: Tb2RevoluteJointDef);
begin
   inherited Create(def);
   m_localAnchor1 := def.localAnchorA;
   m_localAnchor2 := def.localAnchorB;
   m_referenceAngle := def.referenceAngle;

   m_impulse := b2Vec3_Zero;
	 m_motorImpulse := 0.0;

   m_lowerAngle := def.lowerAngle;
   m_upperAngle := def.upperAngle;
   m_maxMotorTorque := def.maxMotorTorque;
   m_motorSpeed := def.motorSpeed;
   m_enableLimit := def.enableLimit;
   m_enableMotor := def.enableMotor;
   m_motorOnBodyB := def.motorOnBodyB;
   m_limitState := e_inactiveLimit;
end;

function Tb2RevoluteJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchor1);
end;

function Tb2RevoluteJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchor2);
end;

function Tb2RevoluteJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   Result.x := m_impulse.x * inv_dt;
   Result.y := m_impulse.y * inv_dt;
end;

function Tb2RevoluteJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := inv_dt * m_impulse.z;
end;

procedure Tb2RevoluteJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   r1, r2, P: TVector2;
   m1, m2, i1, i2, jointAngle: Float;
begin
   if m_enableMotor or m_enableLimit then
   begin
      // You cannot create a rotation limit between bodies that
      // both have fixed rotation.
      //b2Assert(m_bodyA.m_invI > 0.0f || m_bodyB.m_invI > 0.0f);
   end;

   // Compute the effective mass matrix.
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   {$ENDIF}

   // J := [-I -r1_skew I r2_skew]
   //     [ 0       -1 0       1]
   // r_skew := [-ry; rx]

   // Matlab
   // K := [ m1+r1y^2*i1+m2+r2y^2*i2,  -r1y*i1*r1x-r2y*i2*r2x,          -r1y*i1-r2y*i2]
   //     [  -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2,           r1x*i1+r2x*i2]
   //     [          -r1y*i1-r2y*i2,           r1x*i1+r2x*i2,                   i1+i2]

   m1 := m_bodyA.m_invMass;
   m2 := m_bodyB.m_invMass;
   i1 := m_bodyA.m_invI;
   i2 := m_bodyB.m_invI;

   with m_mass do
   begin
      col1.x := m1 + m2 + r1.y * r1.y * i1 + r2.y * r2.y * i2;
      col2.x := -r1.y * r1.x * i1 - r2.y * r2.x * i2;
      col3.x := -r1.y * i1 - r2.y * i2;
      col1.y := col2.x;
      col2.y := m1 + m2 + r1.x * r1.x * i1 + r2.x * r2.x * i2;
      col3.y := r1.x * i1 + r2.x * i2;
      col1.z := col3.x;
      col2.z := col3.y;
      col3.z := i1 + i2;
   end;

   if m_motorOnBodyB then
      m_motorMass := i2
   else
      m_motorMass := i1 + i2;
   if m_motorMass > 0.0 then
      m_motorMass := 1.0 / m_motorMass;

   if not m_enableMotor then
      m_motorImpulse := 0.0;

   if m_enableLimit then
   begin
      jointAngle := m_bodyB.m_sweep.a - m_bodyA.m_sweep.a - m_referenceAngle;
      if Abs(m_upperAngle - m_lowerAngle) < 2.0 * b2_angularSlop then
         m_limitState := e_equalLimits
      else if jointAngle <= m_lowerAngle then
      begin
         if m_limitState <> e_atLowerLimit then
            m_impulse.z := 0.0;
         m_limitState := e_atLowerLimit;
      end
      else if jointAngle >= m_upperAngle then
      begin
         if m_limitState <> e_atUpperLimit then
            m_impulse.z := 0.0;
         m_limitState := e_atUpperLimit;
      end
      else
      begin
         m_limitState := e_inactiveLimit;
         m_impulse.z := 0.0;
      end;
   end
   else
      m_limitState := e_inactiveLimit;

   if step.warmStarting then
   begin
      // Scale impulses to support a variable time step.
      {$IFDEF OP_OVERLOAD}
      m_impulse.MultiplyBy(step.dtRatio);
      {$ELSE}
      MultiplyBy(m_impulse, step.dtRatio);
      {$ENDIF}
      m_motorImpulse := m_motorImpulse * step.dtRatio;

      P.x := m_impulse.x;
      P.y := m_impulse.y;

      {$IFDEF OP_OVERLOAD}
      m_bodyA.m_linearVelocity.SubtractBy(m1 * P);
      m_bodyB.m_linearVelocity.AddBy(m2 * P);
      {$ELSE}
      SubtractBy(m_bodyA.m_linearVelocity, Multiply(P, m1));
      AddBy(m_bodyB.m_linearVelocity, Multiply(P, m2));
      {$ENDIF}

      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity - i1 *
         (b2Cross(r1, P) + m_motorImpulse + m_impulse.z);
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + i2 *
         (b2Cross(r2, P) + m_motorImpulse + m_impulse.z);
   end
   else
   begin
      m_impulse := b2Vec3_Zero;
      m_motorImpulse := 0.0;
   end;
end;

procedure Tb2RevoluteJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   Cdot, impulse: TVector3;
   v1, v2, r1, r2, Cdot1, reduced, P, Cdot2, impulse2: TVector2;
   w1, w2, m1, m2, i1, i2, fCdot, fimpulse, oldImpulse, maxImpulse, newImpulse: Float;
begin
   v1 := m_bodyA.m_linearVelocity;
   w1 := m_bodyA.m_angularVelocity;
   v2 := m_bodyB.m_linearVelocity;
   w2 := m_bodyB.m_angularVelocity;

   m1 := m_bodyA.m_invMass;
   m2 := m_bodyB.m_invMass;
   i1 := m_bodyA.m_invI;
   i2 := m_bodyB.m_invI;

   // Solve motor constraint.
   if m_enableMotor and (m_limitState <> e_equalLimits) then
   begin
      if m_motorOnBodyB then
      begin
         fCdot := w2 - m_motorSpeed;
         fimpulse := m_motorMass * (-fCdot);
         oldImpulse := m_motorImpulse;
         maxImpulse := step.dt * m_maxMotorTorque;
         m_motorImpulse := b2Clamp(m_motorImpulse + fimpulse, -maxImpulse, maxImpulse);
         fimpulse := m_motorImpulse - oldImpulse;
         w2 := w2 + i2 * fimpulse;
      end
      else
      begin
         fCdot := w2 - w1 - m_motorSpeed;
         fimpulse := m_motorMass * (-fCdot);
         oldImpulse := m_motorImpulse;
         maxImpulse := step.dt * m_maxMotorTorque;
         m_motorImpulse := b2Clamp(m_motorImpulse + fimpulse, -maxImpulse, maxImpulse);
         fimpulse := m_motorImpulse - oldImpulse;
         w1 := w1 - i1 * fimpulse;
         w2 := w2 + i2 * fimpulse;
      end;
   end;

   // Solve limit constraint.
   if m_enableLimit and (m_limitState <> e_inactiveLimit) then
   begin
      {$IFDEF OP_OVERLOAD}
      r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
      r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
      {$ELSE}
      r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
      r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
      {$ENDIF}

      // Solve point-to-point constraint
      {$IFDEF OP_OVERLOAD}
      Cdot1 := v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1);
      {$ELSE}
      Cdot1 := Subtract(Add(v2, b2Cross(w2, r2)), Add(v1, b2Cross(w1, r1)));
      {$ENDIF}
      Cdot.x := Cdot1.x;
      Cdot.y := Cdot1.y;
      Cdot.z := w2 - w1;

      {$IFDEF OP_OVERLOAD}
      impulse := m_mass.Solve33(-Cdot);
      {$ELSE}
      impulse := Solve33(m_mass, Negative(Cdot));
      {$ENDIF}

      if m_limitState = e_equalLimits then
         {$IFDEF OP_OVERLOAD}
         m_impulse.AddBy(impulse)
         {$ELSE}
         AddBy(m_impulse, impulse)
         {$ENDIF}
      else if m_limitState = e_atLowerLimit then
      begin
         newImpulse := m_impulse.z + impulse.z;
         if newImpulse < 0.0 then
         begin
            {$IFDEF OP_OVERLOAD}
            reduced := m_mass.Solve22(-Cdot1);
            {$ELSE}
            reduced := Solve22(m_mass, Negative(Cdot1));
            {$ENDIF}
            impulse.x := reduced.x;
            impulse.y := reduced.y;
            impulse.z := -m_impulse.z;
            m_impulse.x := m_impulse.x + reduced.x;
            m_impulse.y := m_impulse.y + reduced.y;
            m_impulse.z := 0.0;
         end;
      end
      else if m_limitState = e_atUpperLimit then
      begin
         newImpulse := m_impulse.z + impulse.z;
         if newImpulse > 0.0 then
         begin
            {$IFDEF OP_OVERLOAD}
            reduced := m_mass.Solve22(-Cdot1);
            {$ELSE}
            reduced := Solve22(m_mass, Negative(Cdot1));
            {$ENDIF}
            impulse.x := reduced.x;
            impulse.y := reduced.y;
            impulse.z := -m_impulse.z;
            m_impulse.x := m_impulse.x + reduced.x;
            m_impulse.y := m_impulse.y + reduced.y;
            m_impulse.z := 0.0;
         end;
      end;

      P.x := impulse.x;
      P.y := impulse.y;

      {$IFDEF OP_OVERLOAD}
      v1.SubtractBy(m1 * P);
      v2.AddBy(m2 * P);
      {$ELSE}
      SubtractBy(v1, Multiply(P, m1));
      AddBy(v2, Multiply(P, m2));
      {$ENDIF}

      w1 := w1 - i1 * (b2Cross(r1, P) + impulse.z);
      w2 := w2 + i2 * (b2Cross(r2, P) + impulse.z);
   end
   else
   begin
      {$IFDEF OP_OVERLOAD}
      r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
      r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
      {$ELSE}
      r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
      r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
      {$ENDIF}

      // Solve point-to-point constraint
      {$IFDEF OP_OVERLOAD}
      Cdot2 := v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1);
      impulse2 := m_mass.Solve22(-Cdot2);
      {$ELSE}
      Cdot2 := Subtract(Add(v2, b2Cross(w2, r2)), Add(v1, b2Cross(w1, r1)));
      impulse2 := Solve22(m_mass, Negative(Cdot2));
      {$ENDIF}

      m_impulse.x := m_impulse.x + impulse2.x;
      m_impulse.y := m_impulse.y + impulse2.y;

      {$IFDEF OP_OVERLOAD}
      v1.SubtractBy(m1 * impulse2);
      v2.AddBy(m2 * impulse2);
      {$ELSE}
      SubtractBy(v1, Multiply(impulse2, m1));
      AddBy(v2, Multiply(impulse2, m2));
      {$ENDIF}

      w1 := w1 - i1 * b2Cross(r1, impulse2);
      w2 := w2 + i2 * b2Cross(r2, impulse2);
   end;

   m_bodyA.m_linearVelocity := v1;
   m_bodyA.m_angularVelocity := w1;
   m_bodyB.m_linearVelocity := v2;
   m_bodyB.m_angularVelocity := w2;
end;

function Tb2RevoluteJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
const
   k_allowedStretch = 10.0 * b2_linearSlop;
   k_beta = 0.5;
var
   angularError, positionError, angle, limitImpulse, C,
   invMass1, invMass2, invI1, invI2, m: Float;
   r1, r2, CV, u, impulse: TVector2;
   K1, K2, K3, K: TMatrix22;
begin
   // TODO_ERIN block solve with limit.
   //B2_NOT_USED(baumgarte);

   angularError := 0.0;
   //positionError := 0.0;

   // Solve angular limit constraint.
   if m_enableLimit and (m_limitState <> e_inactiveLimit) then
   begin
      angle := m_bodyB.m_sweep.a - m_bodyA.m_sweep.a - m_referenceAngle;
      limitImpulse := 0.0;

      if m_limitState = e_equalLimits then
      begin
         // Prevent large angular corrections
         C := b2Clamp(angle - m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
         limitImpulse := -m_motorMass * C;
         angularError := Abs(C);
      end
      else if m_limitState = e_atLowerLimit then
      begin
         C := angle - m_lowerAngle;
         angularError := -C;

         // Prevent large angular corrections and allow some slop.
         C := b2Clamp(C + b2_angularSlop, -b2_maxAngularCorrection, 0.0);
         limitImpulse := -m_motorMass * C;
      end
      else if m_limitState = e_atUpperLimit then
      begin
         C := angle - m_upperAngle;
         angularError := C;

         // Prevent large angular corrections and allow some slop.
         C := b2Clamp(C - b2_angularSlop, 0.0, b2_maxAngularCorrection);
         limitImpulse := -m_motorMass * C;
      end;

      m_bodyA.m_sweep.a := m_bodyA.m_sweep.a - m_bodyA.m_invI * limitImpulse;
      m_bodyB.m_sweep.a := m_bodyB.m_sweep.a + m_bodyB.m_invI * limitImpulse;

      m_bodyA.SynchronizeTransform;
      m_bodyB.SynchronizeTransform;
   end;

   // Solve point-to-point constraint.
   begin
      {$IFDEF OP_OVERLOAD}
      r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
      r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);

      CV := m_bodyB.m_sweep.c + r2 - m_bodyA.m_sweep.c - r1;
      positionError := CV.Length;
      {$ELSE}
      r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
      r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));

      CV := Subtract(Add(m_bodyB.m_sweep.c, r2), Add(m_bodyA.m_sweep.c, r1));
      positionError := LengthVec(CV);
      {$ENDIF}

      invMass1 := m_bodyA.m_invMass;
      invMass2 := m_bodyB.m_invMass;
      invI1 := m_bodyA.m_invI;
      invI2 := m_bodyB.m_invI;

      // Handle large detachment.
      if {$IFDEF OP_OVERLOAD}CV.SqrLength{$ELSE}SqrLength(CV){$ENDIF} >
         k_allowedStretch * k_allowedStretch then
      begin
         // Use a particle solution (no rotation).
         u := CV;
         {$IFDEF OP_OVERLOAD}
         u.Normalize;
         {$ELSE}
         Normalize(u);
         {$ENDIF}
         m := invMass1 + invMass2;
         if m > 0.0 then
            m :=1.0 / m;
         {$IFDEF OP_OVERLOAD}
         impulse := m * (-CV);
         m_bodyA.m_sweep.c.SubtractBy(k_beta * invMass1 * impulse);
         m_bodyB.m_sweep.c.AddBy(k_beta * invMass2 * impulse);
         CV := m_bodyB.m_sweep.c + r2 - m_bodyA.m_sweep.c - r1;
         {$ELSE}
         impulse := Multiply(CV, -m);
         SubtractBy(m_bodyA.m_sweep.c, Multiply(impulse, k_beta * invMass1));
         AddBy(m_bodyB.m_sweep.c, Multiply(impulse, k_beta * invMass2));
         CV := Subtract(Add(m_bodyB.m_sweep.c, r2), Add(m_bodyA.m_sweep.c, r1));
         {$ENDIF}
      end;

      K1.col1.x := invMass1 + invMass2;
      K1.col2.x := 0.0;
      K1.col1.y := 0.0;
      K1.col2.y := invMass1 + invMass2;

      K2.col1.x :=  invI1 * r1.y * r1.y;
      K2.col2.x := -invI1 * r1.x * r1.y;
      K2.col1.y := -invI1 * r1.x * r1.y;
      K2.col2.y :=  invI1 * r1.x * r1.x;

      K3.col1.x :=  invI2 * r2.y * r2.y;
      K3.col2.x := -invI2 * r2.x * r2.y;
      K3.col1.y := -invI2 * r2.x * r2.y;
      K3.col2.y :=  invI2 * r2.x * r2.x;

      {$IFDEF OP_OVERLOAD}
      K := K1 + K2 + K3;
      impulse := K.Solve(-CV);

      m_bodyA.m_sweep.c.SubtractBy(invMass1 * impulse);
      m_bodyB.m_sweep.c.AddBy(invMass2 * impulse);
      {$ELSE}
      K := Add(K1, K2, K3);
      impulse := Solve(K, Negative(CV));

      SubtractBy(m_bodyA.m_sweep.c, Multiply(impulse, invMass1));
      AddBy(m_bodyB.m_sweep.c, Multiply(impulse, m_bodyB.m_invMass));
      {$ENDIF}

      m_bodyA.m_sweep.a := m_bodyA.m_sweep.a - invI1 * b2Cross(r1, impulse);
      m_bodyB.m_sweep.a := m_bodyB.m_sweep.a + invI2 * b2Cross(r2, impulse);

      m_bodyA.SynchronizeTransform;
      m_bodyB.SynchronizeTransform;
   end;

   Result := (positionError <= b2_linearSlop) and (angularError <= b2_angularSlop);
end;

function Tb2RevoluteJoint.GetJointAngle: Float;
begin
	 Result := m_bodyB.m_sweep.a - m_bodyA.m_sweep.a - m_referenceAngle;
end;

function Tb2RevoluteJoint.GetJointSpeed: Float;
begin
	 Result := m_bodyB.m_angularVelocity - m_bodyA.m_angularVelocity;
end;

procedure Tb2RevoluteJoint.EnableLimit(flag: Boolean);
begin
	 m_bodyA.SetAwake(True);
	 m_bodyB.SetAwake(True);
	 m_enableLimit := flag;
end;

procedure Tb2RevoluteJoint.SetLimits(lower, upper: Float);
begin
   //b2Assert(lower <= upper);
	 m_bodyA.SetAwake(True);
	 m_bodyB.SetAwake(True);
   m_lowerAngle := lower;
   m_upperAngle := upper;
end;

procedure Tb2RevoluteJoint.EnableMotor(flag: Boolean);
begin
	 m_bodyA.SetAwake(True);
	 m_bodyB.SetAwake(True);
	 m_enableMotor := flag;
end;

procedure Tb2RevoluteJoint.SetMotorSpeed(speed: Float);
begin
	 m_bodyA.SetAwake(True);
	 m_bodyB.SetAwake(True);
	 m_motorSpeed := speed;
end;

procedure Tb2RevoluteJoint.SetMaxMotorTorque(torque: Float);
begin
	 m_bodyA.SetAwake(True);
	 m_bodyB.SetAwake(True);
   m_maxMotorTorque := torque;
end;

{ Tb2GearJointDef }

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = C0 - (cordinate1 + ratio * coordinate2) = 0
// Cdot = -(Cdot1 + ratio * Cdot2)
// J = -[J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

constructor Tb2GearJointDef.Create;
begin
   inherited;
	 JointType := e_gearJoint;
	 joint1 := nil;
	 joint2 := nil;
	 ratio := 1.0;
end;

{ Tb2GearJoint }

constructor Tb2GearJoint.Create(def: Tb2GearJointDef);
var
   type1, type2: Tb2JointType;
   coordinate1, coordinate2: Float;
begin
   inherited Create(def);
   type1 := def.joint1.m_type;
   type2 := def.joint2.m_type;

   //b2Assert(type1 == e_revoluteJoint || type1 == e_prismaticJoint);
   //b2Assert(type2 == e_revoluteJoint || type2 == e_prismaticJoint);
	 //b2Assert(def->joint1->m_bodyA()->m_type() == b2_staticBody);
	 //b2Assert(def->joint2->m_bodyA()->m_type() == b2_staticBody);

   m_revolute1 := nil;
   m_prismatic1 := nil;
   m_revolute2 := nil;
   m_prismatic2 := nil;

   m_ground1 := def.joint1.m_bodyA;
   m_bodyA := def.joint1.m_bodyB;
   if type1 = e_revoluteJoint then
   begin
      m_revolute1 := Tb2RevoluteJoint(def.joint1);
      m_groundAnchor1 := m_revolute1.m_localAnchor1;
      m_localAnchor1 := m_revolute1.m_localAnchor2;
      coordinate1 := m_revolute1.GetJointAngle;
   end
   else
   begin
      m_prismatic1 := Tb2PrismaticJoint(def.joint1);
      m_groundAnchor1 := m_prismatic1.m_localAnchor1;
      m_localAnchor1 := m_prismatic1.m_localAnchor2;
      coordinate1 := m_prismatic1.GetJointTranslation;
   end;

   m_ground2 := def.joint2.m_bodyA;
   m_bodyB := def.joint2.m_bodyB;
   if type2 = e_revoluteJoint then
   begin
      m_revolute2 := Tb2RevoluteJoint(def.joint2);
      m_groundAnchor2 := m_revolute2.m_localAnchor1;
      m_localAnchor2 := m_revolute2.m_localAnchor2;
      coordinate2 := m_revolute2.GetJointAngle;
   end
   else
   begin
      m_prismatic2 := Tb2PrismaticJoint(def.joint2);
      m_groundAnchor2 := m_prismatic2.m_localAnchor1;
      m_localAnchor2 := m_prismatic2.m_localAnchor2;
      coordinate2 := m_prismatic2.GetJointTranslation;
   end;

   m_ratio := def.ratio;
   m_constant := coordinate1 + m_ratio * coordinate2;
   m_impulse := 0.0;
end;

function Tb2GearJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchor1);
end;

function Tb2GearJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchor2);
end;

function Tb2GearJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   // TODO_ERIN not tested
   {$IFDEF OP_OVERLOAD}
   Result := (inv_dt * m_impulse) * m_J.linearB;
   {$ELSE}
   Result := Multiply(m_J.linearB, inv_dt * m_impulse);
   {$ENDIF}
end;

function Tb2GearJoint.GetReactionTorque(inv_dt: Float): Float;
var
   r, P: TVector2;
begin
   // TODO_ERIN not tested
   {$IFDEF OP_OVERLOAD}
   r := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   P := m_impulse * m_J.linearB;
   {$ELSE}
   r := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   P := Multiply(m_J.linearB, m_impulse);
   {$ENDIF}
   Result := inv_dt * m_impulse * m_J.angularB - b2Cross(r, P);
end;

procedure Tb2GearJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   K, crug: Float;
   ug, r: TVector2;
begin
   K := 0.0;
   {$IFDEF OP_OVERLOAD}
   m_J.SetZero;
   {$ELSE}
   SetZero(m_J);
   {$ENDIF}

   if Assigned(m_revolute1) then
   begin
      m_J.angularA := -1.0;
      K := K + m_bodyA.m_invI;
   end
   else
   begin
      ug := b2Mul(m_ground1.m_xf.R, m_prismatic1.m_localXAxis1);
      {$IFDEF OP_OVERLOAD}
      r := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
      m_J.linearA := -ug;
      {$ELSE}
      r := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
      m_J.linearA := Negative(ug);
      {$ENDIF}
      crug := b2Cross(r, ug);
      m_J.angularA := -crug;
      K := K + m_bodyA.m_invMass + m_bodyA.m_invI * crug * crug;
   end;

   if Assigned(m_revolute2) then
   begin
      m_J.angularB := -m_ratio;
      K := K + m_ratio * m_ratio * m_bodyB.m_invI;
   end
   else
   begin
      ug := b2Mul(m_ground2.m_xf.R, m_prismatic2.m_localXAxis1);
      {$IFDEF OP_OVERLOAD}
      r := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
      m_J.linearB := -m_ratio * ug;
      {$ELSE}
      r := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
      m_J.linearB := Multiply(ug, -m_ratio);
      {$ENDIF}
      crug := b2Cross(r, ug);
      m_J.angularB := -m_ratio * crug;
      K := K + m_ratio * m_ratio * (m_bodyB.m_invMass + m_bodyB.m_invI * crug * crug);
   end;

   // Compute effective mass.
   //b2Assert(K > 0.0);
   if K > 0.0 then
      m_mass := 1.0 / K
   else
      m_mass := 0.0;

   if step.warmStarting then
   begin
      // Warm starting.
      {$IFDEF OP_OVERLOAD}
      m_bodyA.m_linearVelocity.AddBy(m_bodyA.m_invMass * m_impulse * m_J.linearA);
      m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * m_impulse * m_J.linearB);
      {$ELSE}
      AddBy(m_bodyA.m_linearVelocity, Multiply(m_J.linearA, m_bodyA.m_invMass * m_impulse));
      AddBy(m_bodyB.m_linearVelocity, Multiply(m_J.linearB, m_bodyB.m_invMass * m_impulse));
      {$ENDIF}
      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity + m_bodyA.m_invI * m_impulse * m_J.angularA;
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * m_impulse * m_J.angularB;
   end
   else
      m_impulse := 0.0;
end;

procedure Tb2GearJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   Cdot, impulse: Float;
begin
   {$IFDEF OP_OVERLOAD}
   Cdot := m_J.Compute(m_bodyA.m_linearVelocity, m_bodyB.m_linearVelocity,
   m_bodyA.m_angularVelocity, m_bodyB.m_angularVelocity);
   {$ELSE}
   Cdot := Compute(m_J,	m_bodyA.m_linearVelocity, m_bodyB.m_linearVelocity,
   m_bodyA.m_angularVelocity, m_bodyB.m_angularVelocity);
   {$ENDIF}

	 impulse := m_mass * (-Cdot);
	 m_impulse := m_impulse + impulse;

   {$IFDEF OP_OVERLOAD}
   m_bodyA.m_linearVelocity.AddBy(m_bodyA.m_invMass * impulse * m_J.linearA);
   m_bodyB.m_linearVelocity.AddBy(m_bodyB.m_invMass * impulse * m_J.linearB);
   {$ELSE}
   AddBy(m_bodyA.m_linearVelocity, Multiply(m_J.linearA, m_bodyA.m_invMass * impulse));
   AddBy(m_bodyB.m_linearVelocity, Multiply(m_J.linearB, m_bodyB.m_invMass * impulse));
   {$ENDIF}
   m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity + m_bodyA.m_invI * impulse * m_J.angularA;
   m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * impulse * m_J.angularB;
end;

function Tb2GearJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
var
   linearError, coordinate1, coordinate2, C, impulse: Float;
begin
   linearError := 0.0;

   if Assigned(m_revolute1) then
      coordinate1 := m_revolute1.GetJointAngle
   else
      coordinate1 := m_prismatic1.GetJointTranslation;

   if Assigned(m_revolute2) then
      coordinate2 := m_revolute2.GetJointAngle
   else
      coordinate2 := m_prismatic2.GetJointTranslation;

   C := m_constant - (coordinate1 + m_ratio * coordinate2);
   impulse := m_mass * (-C);

   {$IFDEF OP_OVERLOAD}
   m_bodyA.m_sweep.c.AddBy(m_bodyA.m_invMass * impulse * m_J.linearA);
   m_bodyB.m_sweep.c.AddBy(m_bodyB.m_invMass * impulse * m_J.linearB);
   {$ELSE}
   AddBy(m_bodyA.m_sweep.c, Multiply(m_J.linearA, m_bodyA.m_invMass * impulse));
   AddBy(m_bodyB.m_sweep.c, Multiply(m_J.linearB, m_bodyB.m_invMass * impulse));
   {$ENDIF}
   m_bodyA.m_sweep.a := m_bodyA.m_sweep.a + m_bodyA.m_invI * impulse * m_J.angularA;
   m_bodyB.m_sweep.a := m_bodyB.m_sweep.a + m_bodyB.m_invI * impulse * m_J.angularB;

   m_bodyA.SynchronizeTransform;
   m_bodyB.SynchronizeTransform;

   // TODO_ERIN not implemented
   Result := linearError < b2_linearSlop;
end;

{ Tb2FrictionJointDef }

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

constructor Tb2FrictionJointDef.Create;
begin
   inherited;
   JointType := e_frictionJoint;
   localAnchorA := b2Vec2_Zero;
   localAnchorB := b2Vec2_Zero;
   maxForce := 0.0;
   maxTorque := 0.0;
end;

procedure Tb2FrictionJointDef.Initialize(bodyA, bodyB: Tb2Body; const anchor: TVector2);
begin
   Self.bodyA := bodyA;
   Self.bodyB := bodyB;
   localAnchorA := bodyA.GetLocalPoint(anchor);
   localAnchorB := bodyB.GetLocalPoint(anchor);
end;

{ Tb2FrictionJoint }

constructor Tb2FrictionJoint.Create(def: Tb2FrictionJointDef);
begin
   inherited Create(def);
   m_localAnchorA := def.localAnchorA;
   m_localAnchorB := def.localAnchorB;

   m_linearImpulse := b2Vec2_Zero;
   m_angularImpulse := 0.0;

   m_maxForce := def.maxForce;
   m_maxTorque := def.maxTorque;
end;

function Tb2FrictionJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchorA);
end;

function Tb2FrictionJoint.GetAnchorB: TVector2;
begin
   Result  := m_bodyB.GetWorldPoint(m_localAnchorB);
end;

function Tb2FrictionJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := inv_dt * m_linearImpulse;
   {$ELSE}
   Result := Multiply(m_linearImpulse, inv_dt);
   {$ENDIF}
end;

function Tb2FrictionJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := inv_dt * m_angularImpulse;
end;

procedure Tb2FrictionJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   rA, rB: TVector2;
   mA, mB, iA, iB: Float;
   K1, K2, K3, K: TMatrix22;
begin
   // Compute the effective mass matrix.
   {$IFDEF OP_OVERLOAD}
	 rA := b2Mul(m_bodyA.m_xf.R, m_localAnchorA - m_bodyA.GetLocalCenter);
	 rB := b2Mul(m_bodyB.m_xf.R, m_localAnchorB - m_bodyA.GetLocalCenter);
   {$ELSE}
   rA := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchorA, m_bodyA.GetLocalCenter));
   rB := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchorB, m_bodyB.GetLocalCenter));
   {$ENDIF}

   // J := [-I -r1_skew I r2_skew]
   //     [ 0       -1 0       1]
   // r_skew := [-ry; rx]

   // Matlab
   // K := [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
   //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
   //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

   mA := m_bodyA.m_invMass;
   mB := m_bodyB.m_invMass;
   iA := m_bodyA.m_invI;
   iB := m_bodyB.m_invI;

   K1.col1.x := mA + mB;
   K1.col2.x := 0.0;
   K1.col1.y := 0.0;
   K1.col2.y := mA + mB;

   K2.col1.x :=  iA * rA.y * rA.y;
   K2.col2.x := -iA * rA.x * rA.y;
   K2.col1.y := -iA * rA.x * rA.y;
   K2.col2.y :=  iA * rA.x * rA.x;

   K3.col1.x :=  iB * rB.y * rB.y;
   K3.col2.x := -iB * rB.x * rB.y;
   K3.col1.y := -iB * rB.x * rB.y;
   K3.col2.y :=  iB * rB.x * rB.x;

   {$IFDEF OP_OVERLOAD}
   K := K1 + K2 + K3;
   m_linearMass := K.GetInverse;
   {$ELSE}
   K := Add(K1, K2, K3);
   m_linearMass := GetInverse(K);
   {$ENDIF}

   m_angularMass := iA + iB;
   if m_angularMass > 0.0 then
      m_angularMass := 1.0 / m_angularMass;

   if step.warmStarting then
   begin
      // Scale impulses to support a variable time step.
      {$IFDEF OP_OVERLOAD}
      m_linearImpulse.MultiplyBy(step.dtRatio);
      {$ELSE}
      MultiplyBy(m_linearImpulse, step.dtRatio);
      {$ENDIF}
      m_angularImpulse := m_angularImpulse * step.dtRatio;

      {$IFDEF OP_OVERLOAD}
      m_bodyA.m_linearVelocity.SubtractBy(mA * m_linearImpulse);
      m_bodyB.m_linearVelocity.AddBy(mB * m_linearImpulse);
      {$ELSE}
      SubtractBy(m_bodyA.m_linearVelocity, Multiply(m_linearImpulse, mA));
      AddBy(m_bodyB.m_linearVelocity, Multiply(m_linearImpulse, mB));
      {$ENDIF}

      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity -
         iA * (b2Cross(rA, m_linearImpulse) + m_angularImpulse);
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity +
         iB * (b2Cross(rB, m_linearImpulse) + m_angularImpulse);
   end
   else
   begin
      m_linearImpulse := b2Vec2_Zero;
      m_angularImpulse := 0.0;
   end;
end;

procedure Tb2FrictionJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   vA, vB, rA, rB, Cdot, impulse, oldImpulse: TVector2;
   wA, wB, fCdot, impulsef, oldImpulsef, maxImpulse: Float;
begin
   //B2_NOT_USED(step);

   vA := m_bodyA.m_linearVelocity;
   wA := m_bodyA.m_angularVelocity;
   vB := m_bodyB.m_linearVelocity;
   wB := m_bodyB.m_angularVelocity;

   {$IFDEF OP_OVERLOAD}
   rA := b2Mul(m_bodyA.m_xf.R, m_localAnchorA - m_bodyA.GetLocalCenter);
   rB := b2Mul(m_bodyB.m_xf.R, m_localAnchorB - m_bodyB.GetLocalCenter);
   {$ELSE}
   rA := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchorA, m_bodyA.GetLocalCenter));
   rB := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchorB, m_bodyB.GetLocalCenter));
   {$ENDIF}

   // Solve angular friction
   begin
      fCdot := wB - wA;
      impulsef := -m_angularMass * fCdot;

      oldImpulsef := m_angularImpulse;
      maxImpulse := step.dt * m_maxTorque;
      m_angularImpulse := b2Clamp(m_angularImpulse + impulsef, -maxImpulse, maxImpulse);
      impulsef := m_angularImpulse - oldImpulsef;

      wA := wA - m_bodyA.m_invI * impulsef;
      wB := wB + m_bodyB.m_invI * impulsef;
   end;

   // Solve linear friction
   begin
      {$IFDEF OP_OVERLOAD}
      Cdot := vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA);
      impulse := -b2Mul(m_linearMass, Cdot);
      {$ELSE}
      Cdot := Subtract(Add(vB, b2Cross(wB, rB)), Add(vA, b2Cross(wA, rA)));
      impulse := Negative(b2Mul(m_linearMass, Cdot));
      {$ENDIF}

      oldImpulse := m_linearImpulse;
      {$IFDEF OP_OVERLOAD}
      m_linearImpulse.AddBy(impulse);
      {$ELSE}
      AddBy(m_linearImpulse, impulse);
      {$ENDIF}

      maxImpulse := step.dt * m_maxForce;

      if
      {$IFDEF OP_OVERLOAD}
      m_linearImpulse.SqrLength > maxImpulse * maxImpulse
      {$ELSE}
      SqrLength(m_linearImpulse) > maxImpulse * maxImpulse
      {$ENDIF}
      then
      begin
         {$IFDEF OP_OVERLOAD}
         m_linearImpulse.Normalize;
         m_linearImpulse.MultiplyBy(maxImpulse);
         {$ELSE}
         Normalize(m_linearImpulse);
         MultiplyBy(m_linearImpulse, maxImpulse);
         {$ENDIF}
      end;

      {$IFDEF OP_OVERLOAD}
      impulse := m_linearImpulse - oldImpulse;
      vA.SubtractBy(m_bodyA.m_invMass * impulse);
      vB.AddBy(m_bodyB.m_invMass * impulse);
      {$ELSE}
      impulse := Subtract(m_linearImpulse, oldImpulse);
      SubtractBy(vA, Multiply(impulse, m_bodyA.m_invMass));
      AddBy(vB, Multiply(impulse, m_bodyB.m_invMass));
      {$ENDIF}
      wA := wA - m_bodyA.m_invI * b2Cross(rA, impulse);
      wB := wB + m_bodyB.m_invI * b2Cross(rB, impulse);
   end;

   m_bodyA.m_linearVelocity := vA;
   m_bodyA.m_angularVelocity := wA;
   m_bodyB.m_linearVelocity := vB;
   m_bodyB.m_angularVelocity := wB;
end;

function Tb2FrictionJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
begin
	 //B2_NOT_USED(baumgarte);
   Result := True;
end;

{ Tb2LineJointDef }

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)


// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(2) = max(f2(2), 0)
// upper: f2(2) = min(f2(2), 0)
//
// Solve for correct f2(1)
// K(1,1) * f2(1) = -Cdot(1) - K(1,2) * f2(2) + K(1,1:2) * f1
//                = -Cdot(1) - K(1,2) * f2(2) + K(1,1) * f1(1) + K(1,2) * f1(2)
// K(1,1) * f2(1) = -Cdot(1) - K(1,2) * (f2(2) - f1(2)) + K(1,1) * f1(1)
// f2(1) = invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
//
// Now compute impulse to be applied:
// df = f2 - f1

constructor Tb2LineJointDef.Create;
begin
   inherited;
   JointType := e_lineJoint;
   localAnchorA := b2Vec2_Zero;
   localAnchorB := b2Vec2_Zero;
   SetValue(localAxisA, 1.0, 0.0);
   enableLimit := False;
   lowerTranslation := 0.0;
   upperTranslation := 0.0;
   enableMotor := False;
   maxMotorForce := 0.0;
   motorSpeed := 0.0;
end;

procedure Tb2LineJointDef.Initialize(bodyA, bodyB: Tb2Body; const anchor, axis: TVector2);
begin
   Self.bodyA := bodyA;
   Self.bodyB := bodyB;
   localAnchorA := bodyA.GetLocalPoint(anchor);
   localAnchorB := bodyB.GetLocalPoint(anchor);
   localAxisA := bodyA.GetLocalVector(axis);
end;

{ Tb2LineJoint }

constructor Tb2LineJoint.Create(def: Tb2LineJointDef);
begin
   inherited Create(def);
   m_localAnchor1 := def.localAnchorA;
   m_localAnchor2 := def.localAnchorB;
   m_localXAxis1 := def.localAxisA;
   m_localYAxis1 := b2Cross(1.0, m_localXAxis1);

   m_impulse := b2Vec2_Zero;
   m_motorMass := 0.0;
   m_motorImpulse := 0.0;

   m_lowerTranslation := def.lowerTranslation;
   m_upperTranslation := def.upperTranslation;
   m_maxMotorForce := def.maxMotorForce;
   m_motorSpeed := def.motorSpeed;
   m_enableLimit := def.enableLimit;
   m_enableMotor := def.enableMotor;
   m_limitState := e_inactiveLimit;

   m_axis := b2Vec2_Zero;
   m_perp := b2Vec2_Zero;
end;

function Tb2LineJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchor1);
end;

function Tb2LineJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchor2);
end;

function Tb2LineJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.y) * m_axis);
   {$ELSE}
   Result := Multiply(Add(Multiply(m_perp, m_impulse.x), Multiply(m_axis, m_motorImpulse + m_impulse.y)), inv_dt);
   {$ENDIF}
end;

function Tb2LineJoint.GetReactionTorque(inv_dt: Float): Float;
begin
	 //B2_NOT_USED(inv_dt);
	 Result := 0.0;
end;

function Tb2LineJoint.GetJointSpeed: Float;
var
   r1, r2, d, axis: TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   d := (m_bodyB.m_sweep.c + r2) - (m_bodyA.m_sweep.c + r1);
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   d := Subtract(Add(m_bodyB.m_sweep.c, r2), Add(m_bodyA.m_sweep.c, r1));
   {$ENDIF}
   axis := m_bodyA.GetWorldVector(m_localXAxis1);

   {$IFDEF OP_OVERLOAD}
   Result := b2Dot(d, b2Cross(m_bodyA.m_angularVelocity, axis)) +
      b2Dot(axis, m_bodyB.m_linearVelocity + b2Cross(m_bodyB.m_angularVelocity, r2) -
      m_bodyA.m_linearVelocity - b2Cross(m_bodyA.m_angularVelocity, r1));
   {$ELSE}
   Result := b2Dot(d, b2Cross(m_bodyA.m_angularVelocity, axis)) +
      b2Dot(axis, Subtract(Add(m_bodyB.m_linearVelocity,
      b2Cross(m_bodyB.m_angularVelocity, r2)), Add(m_bodyA.m_linearVelocity,
      b2Cross(m_bodyA.m_angularVelocity, r1))));
   {$ENDIF}
end;

function Tb2LineJoint.GetJointTranslation: Float;
begin
   {$IFDEF OP_OVERLOAD}
   Result := b2Dot(m_bodyB.GetWorldPoint(m_localAnchor2) -
      m_bodyA.GetWorldPoint(m_localAnchor1), m_bodyA.GetWorldVector(m_localXAxis1));
   {$ELSE}
   Result := b2Dot(Subtract(m_bodyB.GetWorldPoint(m_localAnchor2),
      m_bodyA.GetWorldPoint(m_localAnchor1)), m_bodyA.GetWorldVector(m_localXAxis1));
   {$ENDIF}
end;

procedure Tb2LineJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   r1, r2, d, P: TVector2;
   k11, k12, k22, jointTranslation, L1, L2: Float;
begin
   // Compute the effective masses.
   {$IFDEF OP_OVERLOAD}
   r1 := b2Mul(m_bodyA.m_xf.R, m_localAnchor1 - m_bodyA.GetLocalCenter);
   r2 := b2Mul(m_bodyB.m_xf.R, m_localAnchor2 - m_bodyB.GetLocalCenter);
   d := m_bodyB.m_sweep.c + r2 - m_bodyA.m_sweep.c - r1;
   {$ELSE}
   r1 := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchor1, m_bodyA.GetLocalCenter));
   r2 := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchor2, m_bodyB.GetLocalCenter));
   d := Subtract(Add(m_bodyB.m_sweep.c, r2), Add(m_bodyA.m_sweep.c, r1));
   {$ENDIF}

   m_invMassA := m_bodyA.m_invMass;
   m_invIA := m_bodyA.m_invI;
   m_invMassB := m_bodyB.m_invMass;
   m_invIB := m_bodyB.m_invI;

   // Compute motor Jacobian and effective mass.
   begin
      m_axis := b2Mul(m_bodyA.m_xf.R, m_localXAxis1);
      {$IFDEF OP_OVERLOAD}
      m_a1 := b2Cross(d + r1, m_axis);
      {$ELSE}
      m_a1 := b2Cross(Add(d, r1), m_axis);
      {$ENDIF}
      m_a2 := b2Cross(r2, m_axis);

      m_motorMass := m_invMassA + m_invMassB +
         m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;
      if m_motorMass > FLT_epsilon then
         m_motorMass := 1.0 / m_motorMass
      else
         m_motorMass := 0.0;
   end;

   // Prismatic constraint.
   begin
      m_perp := b2Mul(m_bodyA.m_xf.R, m_localYAxis1);

      {$IFDEF OP_OVERLOAD}
      m_s1 := b2Cross(d + r1, m_perp);
      {$ELSE}
      m_s1 := b2Cross(Add(d, r1), m_perp);
      {$ENDIF}
      m_s2 := b2Cross(r2, m_perp);

      k11 := m_invMassA + m_invMassB + m_invIA * m_s1 * m_s1 + m_invIB * m_s2 * m_s2;
      k12 := m_invIA * m_s1 * m_a1 + m_invIB * m_s2 * m_a2;
      k22 := m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;

      {$IFDEF OP_OVERLOAD}
      m_K.col1.SetValue(k11, k12);
      m_K.col2.SetValue(k12, k22);
      {$ELSE}
      SetValue(m_K.col1, k11, k12);
      SetValue(m_K.col2, k12, k22);
      {$ENDIF}
   end;

   // Compute motor and limit terms.
   if m_enableLimit then
   begin
      jointTranslation := b2Dot(m_axis, d);
      if Abs(m_upperTranslation - m_lowerTranslation) < 2.0 * b2_linearSlop then
         m_limitState := e_equalLimits
      else if jointTranslation <= m_lowerTranslation then
      begin
        if m_limitState <> e_atLowerLimit then
        begin
           m_limitState := e_atLowerLimit;
           m_impulse.y := 0.0;
        end;
      end
      else if jointTranslation >= m_upperTranslation then
      begin
         if m_limitState <> e_atUpperLimit then
         begin
            m_limitState := e_atUpperLimit;
            m_impulse.y := 0.0;
         end;
      end
      else
      begin
         m_limitState := e_inactiveLimit;
         m_impulse.y := 0.0;
      end;
   end
   else
      m_limitState := e_inactiveLimit;

   if not m_enableMotor then
      m_motorImpulse := 0.0;

   if step.warmStarting then
   begin
      // Account for variable time step.
      {$IFDEF OP_OVERLOAD}
      m_impulse.MultiplyBy(step.dtRatio);
      {$ELSE}
      MultiplyBy(m_impulse, step.dtRatio);
      {$ENDIF}
      m_motorImpulse := m_motorImpulse * step.dtRatio;

      {$IFDEF OP_OVERLOAD}
      P := m_impulse.x * m_perp + (m_motorImpulse + m_impulse.y) * m_axis;
      {$ELSE}
      P := Add(Multiply(m_perp, m_impulse.x), Multiply(m_axis, m_motorImpulse + m_impulse.y));
      {$ENDIF}
      L1 := m_impulse.x * m_s1 + (m_motorImpulse + m_impulse.y) * m_a1;
      L2 := m_impulse.x * m_s2 + (m_motorImpulse + m_impulse.y) * m_a2;

      {$IFDEF OP_OVERLOAD}
      m_bodyA.m_linearVelocity.SubtractBy(m_invMassA * P);
      m_bodyB.m_linearVelocity.AddBy(m_invMassB * P);
      {$ELSE}
      SubtractBy(m_bodyA.m_linearVelocity, Multiply(P, m_invMassA));
      AddBy(m_bodyB.m_linearVelocity, Multiply(P, m_invMassB));
      {$ENDIF}

      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity - m_bodyA.m_invI * L1;
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + m_bodyB.m_invI * L2;
   end
   else
   begin
      m_impulse := b2Vec2_Zero;
      m_motorImpulse := 0.0;
   end;
end;

procedure Tb2LineJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   v1, v2, P, Cdot, f1, dfv: TVector2;
   w1, w2, Cdotf, Cdot2, impulse, oldImpulse, maxImpulse, L1, L2,
   b, f2r, df: Float;
begin
   v1 := m_bodyA.m_linearVelocity;
   w1 := m_bodyA.m_angularVelocity;
   v2 := m_bodyB.m_linearVelocity;
   w2 := m_bodyB.m_angularVelocity;

   // Solve linear motor constraint.
   if m_enableMotor and (m_limitState <> e_equalLimits) then
   begin
      {$IFDEF OP_OVERLOAD}
      Cdotf := b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
      {$ELSE}
      Cdotf := b2Dot(m_axis, Subtract(v2, v1)) + m_a2 * w2 - m_a1 * w1;
      {$ENDIF}
      impulse := m_motorMass * (m_motorSpeed - Cdotf);
      oldImpulse := m_motorImpulse;
      maxImpulse := step.dt * m_maxMotorForce;
      m_motorImpulse := b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse := m_motorImpulse - oldImpulse;

      {$IFDEF OP_OVERLOAD}
      P := impulse * m_axis;
      {$ELSE}
      P := Multiply(m_axis, impulse);
      {$ENDIF}
      L1 := impulse * m_a1;
      L2 := impulse * m_a2;

      {$IFDEF OP_OVERLOAD}
	   	v1.SubtractBy(m_invMassA * P);
   		v2.AddBy(m_invMassB * P);
      {$ELSE}
	   	SubtractBy(v1, Multiply(P, m_invMassA));
   		AddBy(v2, Multiply(P, m_invMassB));
      {$ENDIF}

      w1 := w1 - m_invIA * L1;
   		w2 := w2 + m_invIB * L2;
   end;

   {$IFDEF OP_OVERLOAD}
   Cdotf := b2Dot(m_perp, v2 - v1) + m_s2 * w2 - m_s1 * w1;
   {$ELSE}
   Cdotf := b2Dot(m_perp, Subtract(v2, v1)) + m_s2 * w2 - m_s1 * w1;
   {$ENDIF}

   if m_enableLimit and (m_limitState <> e_inactiveLimit) then
   begin
      // Solve prismatic and limit constraint in block form.
      {$IFDEF OP_OVERLOAD}
      Cdot2 := b2Dot(m_axis, v2 - v1) + m_a2 * w2 - m_a1 * w1;
      {$ELSE}
      Cdot2 := b2Dot(m_axis, Subtract(v2, v1)) + m_a2 * w2 - m_a1 * w1;
      {$ENDIF}
      Cdot.x := Cdotf;
      Cdot.y := Cdot2;

      f1 := m_impulse;
      {$IFDEF OP_OVERLOAD}
      m_impulse.AddBy(m_K.Solve(-Cdot));
      {$ELSE}
      AddBy(m_impulse, Solve(m_K, Negative(Cdot)));
      {$ENDIF}

      if m_limitState = e_atLowerLimit then
         m_impulse.y := b2Max(m_impulse.y, 0.0)
      else if m_limitState = e_atUpperLimit then
         m_impulse.y := b2Min(m_impulse.y, 0.0);

      // f2(1) := invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
      b := -Cdotf - (m_impulse.y - f1.y) * m_K.col2.x;
      if m_K.col1.x <> 0.0 then
         f2r := b / m_K.col1.x + f1.x
      else
         f2r := f1.x;

      m_impulse.x := f2r;
      {$IFDEF OP_OVERLOAD}
      dfv := m_impulse - f1;
      {$ELSE}
      dfv := Subtract(m_impulse, f1);
      {$ENDIF}

      with dfv do
      begin
         {$IFDEF OP_OVERLOAD}
         P := x * m_perp + y * m_axis;
         {$ELSE}
         P := Add(Multiply(m_perp, x), Multiply(m_axis, y));
         {$ENDIF}
         L1 := x * m_s1 + y * m_a1;
         L2 := x * m_s2 + y * m_a2;
      end;

      {$IFDEF OP_OVERLOAD}
      v1.SubtractBy(m_invMassA * P);
      v2.AddBy(m_invMassB * P);
      {$ELSE}
      SubtractBy(v1, Multiply(P, m_invMassA));
      AddBy(v2, Multiply(P, m_invMassB));
      {$ENDIF}

      w1 := w1 - m_invIA * L1;
      w2 := w2 + m_invIB * L2;
   end
   else
   begin
      // Limit is inactive, just solve the prismatic constraint in block form.
      if m_K.col1.x <> 0.0 then
         df := -Cdotf / m_K.col1.x
      else
         df := 0.0;
      m_impulse.x := m_impulse.x + df;

      {$IFDEF OP_OVERLOAD}
      P := df * m_perp;
      {$ELSE}
      P := Multiply(m_perp, df);
      {$ENDIF}
      L1 := df * m_s1;
      L2 := df * m_s2;

      {$IFDEF OP_OVERLOAD}
      v1.SubtractBy(m_invMassA * P);
      v2.AddBy(m_invMassB * P);
      {$ELSE}
      SubtractBy(v1, Multiply(P, m_invMassA));
      AddBy(v2, Multiply(P, m_invMassB));
      {$ENDIF}

      w1 := w1 - m_invIA * L1;
      w2 := w2 + m_invIB * L2;
   end;

   m_bodyA.m_linearVelocity := v1;
   m_bodyA.m_angularVelocity := w1;
   m_bodyB.m_linearVelocity := v2;
   m_bodyB.m_angularVelocity := w2;
end;

function Tb2LineJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
var
   c1, c2, r1, r2, d, impulse, C, P: TVector2;
   a1, a2, linearError, angularError, _C1, _C2, translation,
   k11, k12, k22, L1, L2: Float;
   active: Boolean;
   _R1, _R2: TMatrix22;
begin
   //B2_NOT_USED(baumgarte);

   c1 := m_bodyA.m_sweep.c;
   a1 := m_bodyA.m_sweep.a;
   c2 := m_bodyB.m_sweep.c;
   a2 := m_bodyB.m_sweep.a;

   // Solve linear limit constraint.
   linearError := 0.0;
   //angularError := 0.0;
   active := False;
   _C2 := 0.0;

   {$IFDEF OP_OVERLOAD}
   _R1.SetValue(a1);
   _R2.SetValue(a2);

   r1 := b2Mul(_R1, m_localAnchor1 - m_localCenterA);
   r2 := b2Mul(_R2, m_localAnchor2 - m_localCenterB);
   d := c2 + r2 - c1 - r1;
   {$ELSE}
   SetValue(_R1, a1);
   SetValue(_R2, a2);

   r1 := b2Mul(_R1, Subtract(m_localAnchor1, m_localCenterA));
   r2 := b2Mul(_R2, Subtract(m_localAnchor2, m_localCenterB));
   d := Subtract(Add(c2, r2), Add(c1, r1));
   {$ENDIF}

   if m_enableLimit then
   begin
      m_axis := b2Mul(_R1, m_localXAxis1);

      {$IFDEF OP_OVERLOAD}
      m_a1 := b2Cross(d + r1, m_axis);
      {$ELSE}
      m_a1 := b2Cross(Add(d, r1), m_axis);
      {$ENDIF}
      m_a2 := b2Cross(r2, m_axis);

      translation := b2Dot(m_axis, d);
      if Abs(m_upperTranslation - m_lowerTranslation) < 2.0 * b2_linearSlop then
      begin
         // Prevent large angular corrections
         _C2 := b2Clamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
         linearError := Abs(translation);
         active := True;
      end
      else if translation <= m_lowerTranslation then
      begin
         // Prevent large linear corrections and allow some slop.
         _C2 := b2Clamp(translation - m_lowerTranslation + b2_linearSlop, -b2_maxLinearCorrection, 0.0);
         linearError := m_lowerTranslation - translation;
         active := True;
      end
      else if translation >= m_upperTranslation then
      begin
         // Prevent large linear corrections and allow some slop.
         _C2 := b2Clamp(translation - m_upperTranslation - b2_linearSlop, 0.0, b2_maxLinearCorrection);
         linearError := translation - m_upperTranslation;
         active := True;
      end;
   end;

   m_perp := b2Mul(_R1, m_localYAxis1);

   {$IFDEF OP_OVERLOAD}
   m_s1 := b2Cross(d + r1, m_perp);
   {$ELSE}
   m_s1 := b2Cross(Add(d, r1), m_perp);
   {$ENDIF}
   m_s2 := b2Cross(r2, m_perp);

   _C1 := b2Dot(m_perp, d);

   linearError := b2Max(linearError, Abs(_C1));
   angularError := 0.0;

   if active then
   begin
      k11 := m_invMassA + m_invMassB + m_invIA * m_s1 * m_s1 + m_invIB * m_s2 * m_s2;
      k12 := m_invIA * m_s1 * m_a1 + m_invIB * m_s2 * m_a2;
      k22 := m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;

      {$IFDEF OP_OVERLOAD}
      m_K.col1.SetValue(k11, k12);
      m_K.col2.SetValue(k12, k22);
      {$ELSE}
      SetValue(m_K.col1, k11, k12);
      SetValue(m_K.col2, k12, k22);
      {$ENDIF}

      C.x := _C1;
      C.y := _C2;

      {$IFDEF OP_OVERLOAD}
      impulse := m_K.Solve(-C);
      {$ELSE}
      impulse := Solve(m_K, Negative(C));
      {$ENDIF}
   end
   else
   begin
      k11 := m_invMassA + m_invMassB + m_invIA * m_s1 * m_s1 + m_invIB * m_s2 * m_s2;

      if k11 <> 0.0 then
         impulse.x := - _C1 / k11
      else
         impulse.x := 0.0;
      impulse.y := 0.0;
   end;

   {$IFDEF OP_OVERLOAD}
   P := impulse.x * m_perp + impulse.y * m_axis;
   {$ELSE}
   P := Add(Multiply(m_perp, impulse.x), Multiply(m_axis, impulse.y));
   {$ENDIF}
   L1 := impulse.x * m_s1 + impulse.y * m_a1;
   L2 := impulse.x * m_s2 + impulse.y * m_a2;

   {$IFDEF OP_OVERLOAD}
   c1.SubtractBy(m_invMassA * P);
   c2.AddBy(m_invMassB * P);
   {$ELSE}
   SubtractBy(c1, Multiply(P, m_invMassA));
   AddBy(c2, Multiply(P, m_invMassB));
   {$ENDIF}

   a1 := a1 - m_invIA * L1;
   a2 := a2 + m_invIB * L2;

   // TODO_ERIN remove need for this.
   m_bodyA.m_sweep.c := c1;
   m_bodyA.m_sweep.a := a1;
   m_bodyB.m_sweep.c := c2;
   m_bodyB.m_sweep.a := a2;
   m_bodyA.SynchronizeTransform;
   m_bodyB.SynchronizeTransform;

   Result := (linearError <= b2_linearSlop) and (angularError <= b2_angularSlop);
end;

procedure Tb2LineJoint.EnableLimit(flag: Boolean);
begin
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
   m_enableLimit := flag;
end;

procedure Tb2LineJoint.SetLimits(lower, upper: Float);
begin
   //b2Assert(lower <= upper);
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
   m_lowerTranslation := lower;
   m_upperTranslation := upper;
end;

procedure Tb2LineJoint.EnableMotor(flag: Boolean);
begin
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
   m_enableMotor := flag;
end;

procedure Tb2LineJoint.SetMotorSpeed(speed: Float);
begin
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
   m_motorSpeed := speed;
end;

procedure Tb2LineJoint.SetMaxMotorForce(force: Float);
begin
   m_bodyA.SetAwake(True);
   m_bodyB.SetAwake(True);
   m_maxMotorForce := force;
end;

{ Tb2WeldJointDef }

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

constructor Tb2WeldJointDef.Create;
begin
   inherited;
   JointType := e_weldJoint;
   localAnchorA := b2Vec2_Zero;
   localAnchorB := b2Vec2_Zero;
   referenceAngle := 0.0;
end;

procedure Tb2WeldJointDef.Initialize(bodyA, bodyB: Tb2Body; const anchor: TVector2);
begin
   Self.bodyA := bodyA;
   Self.bodyB := bodyB;
   localAnchorA := bodyA.GetLocalPoint(anchor);
   localAnchorB := bodyB.GetLocalPoint(anchor);
   referenceAngle := bodyB.GetAngle - bodyA.GetAngle;
end;

{ Tb2WeldJoint }

constructor Tb2WeldJoint.Create(def: Tb2WeldJointDef);
begin
   inherited Create(def);
   m_localAnchorA := def.localAnchorA;
   m_localAnchorB := def.localAnchorB;
   m_referenceAngle := def.referenceAngle;
   m_impulse := b2Vec3_Zero;
end;

function Tb2WeldJoint.GetAnchorA: TVector2;
begin
   Result := m_bodyA.GetWorldPoint(m_localAnchorA);
end;

function Tb2WeldJoint.GetAnchorB: TVector2;
begin
   Result := m_bodyB.GetWorldPoint(m_localAnchorB);
end;

function Tb2WeldJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   Result.x := inv_dt * m_impulse.x;
   Result.y := inv_dt * m_impulse.y;
end;

function Tb2WeldJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := inv_dt * m_impulse.z;
end;

procedure Tb2WeldJoint.InitVelocityConstraints(const step: Tb2TimeStep);
var
   rA, rB, P: TVector2;
   mA, mB, iA, iB: Float;
begin
   // Compute the effective mass matrix.
   {$IFDEF OP_OVERLOAD}
   rA := b2Mul(m_bodyA.m_xf.R, m_localAnchorA - m_bodyA.GetLocalCenter);
   rB := b2Mul(m_bodyB.m_xf.R, m_localAnchorB - m_bodyB.GetLocalCenter);
   {$ELSE}
   rA := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchorA, m_bodyA.GetLocalCenter));
   rB := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchorB, m_bodyB.GetLocalCenter));
   {$ENDIF}

   // J := [-I -r1_skew I r2_skew]
   //     [ 0       -1 0       1]
   // r_skew := [-ry; rx]

   // Matlab
   // K := [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
   //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
   //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

   mA := m_bodyA.m_invMass;
   mB := m_bodyB.m_invMass;
   iA := m_bodyA.m_invI;
   iB := m_bodyB.m_invI;

   m_mass.col1.x := mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
   m_mass.col2.x := -rA.y * rA.x * iA - rB.y * rB.x * iB;
   m_mass.col3.x := -rA.y * iA - rB.y * iB;
   m_mass.col1.y := m_mass.col2.x;
   m_mass.col2.y := mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
   m_mass.col3.y := rA.x * iA + rB.x * iB;
   m_mass.col1.z := m_mass.col3.x;
   m_mass.col2.z := m_mass.col3.y;
   m_mass.col3.z := iA + iB;

   if step.warmStarting then
   begin
      // Scale impulses to support a variable time step.
      {$IFDEF OP_OVERLOAD}
      m_impulse.MultiplyBy(step.dtRatio);
      {$ELSE}
      MultiplyBy(m_impulse, step.dtRatio);
      {$ENDIF}

      P.x := m_impulse.x;
      P.y := m_impulse.y;

      {$IFDEF OP_OVERLOAD}
      m_bodyA.m_linearVelocity.SubtractBy(mA * P);
      m_bodyB.m_linearVelocity.AddBy(mB * P);
      {$ELSE}
      SubtractBy(m_bodyA.m_linearVelocity, Multiply(P, mA));
      AddBy(m_bodyB.m_linearVelocity, Multiply(P, mB));
      {$ENDIF}

      m_bodyA.m_angularVelocity := m_bodyA.m_angularVelocity - iA * (b2Cross(rA, P) + m_impulse.z);
      m_bodyB.m_angularVelocity := m_bodyB.m_angularVelocity + iB * (b2Cross(rB, P) + m_impulse.z);
   end
   else
      m_impulse := b2Vec3_Zero;
end;

procedure Tb2WeldJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   vA, vB, rA, rB, Cdot1, P: TVector2;
   wA, wB: Float;
   Cdot, impulse: TVector3;
begin
   //B2_NOT_USED(step);

   vA := m_bodyA.m_linearVelocity;
   wA := m_bodyA.m_angularVelocity;
   vB := m_bodyB.m_linearVelocity;
   wB := m_bodyB.m_angularVelocity;

   {$IFDEF OP_OVERLOAD}
   rA := b2Mul(m_bodyA.m_xf.R, m_localAnchorA - m_bodyA.GetLocalCenter);
   rB := b2Mul(m_bodyB.m_xf.R, m_localAnchorB - m_bodyB.GetLocalCenter);
   {$ELSE}
   rA := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchorA, m_bodyA.GetLocalCenter));
   rB := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchorB, m_bodyB.GetLocalCenter));
   {$ENDIF}

   // Solve point-to-point constraint
   {$IFDEF OP_OVERLOAD}
   Cdot1 := vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA);
   {$ELSE}
   Cdot1 := Subtract(Add(vB, b2Cross(wB, rB)), Add(vA, b2Cross(wA, rA)));
   {$ENDIF}
   Cdot.x := Cdot1.x;
   Cdot.y := Cdot1.y;
   Cdot.z := wB - wA;

   {$IFDEF OP_OVERLOAD}
   impulse := m_mass.Solve33(-Cdot);
   m_impulse.AddBy(impulse);
   {$ELSE}
   impulse := Solve33(m_mass, Negative(Cdot));
   AddBy(m_impulse, impulse);
   {$ENDIF}

   P.x := impulse.x;
   P.y := impulse.y;

   {$IFDEF OP_OVERLOAD}
   vA.SubtractBy(m_bodyA.m_invMass * P);
   vB.AddBy(m_bodyB.m_invMass * P);
   {$ELSE}
   SubtractBy(vA, Multiply(P, m_bodyA.m_invMass));
   AddBy(vB, Multiply(P, m_bodyB.m_invMass));
   {$ENDIF}

   wA := wA - m_bodyA.m_invI * (b2Cross(rA, P) + impulse.z);
   wB := wB + m_bodyB.m_invI * (b2Cross(rB, P) + impulse.z);

   m_bodyA.m_linearVelocity := vA;
   m_bodyA.m_angularVelocity := wA;
   m_bodyB.m_linearVelocity := vB;
   m_bodyB.m_angularVelocity := wB;
end;

function Tb2WeldJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
const
   k_allowedStretch = 10.0 * b2_linearSlop;
var
   mA, mB, iA, iB, C2, positionError, angularError: Float;
   rA, rB, C1, P: TVector2;
   C, impulse: TVector3;
begin
   //B2_NOT_USED(baumgarte);

   mA := m_bodyA.m_invMass;
   mB := m_bodyB.m_invMass;
   iA := m_bodyA.m_invI;
   iB := m_bodyB.m_invI;

   {$IFDEF OP_OVERLOAD}
   rA := b2Mul(m_bodyA.m_xf.R, m_localAnchorA - m_bodyA.GetLocalCenter);
   rB := b2Mul(m_bodyB.m_xf.R, m_localAnchorB - m_bodyB.GetLocalCenter);
   C1 :=  m_bodyB.m_sweep.c + rB - m_bodyA.m_sweep.c - rA;
   {$ELSE}
   rA := b2Mul(m_bodyA.m_xf.R, Subtract(m_localAnchorA, m_bodyA.GetLocalCenter));
   rB := b2Mul(m_bodyB.m_xf.R, Subtract(m_localAnchorB, m_bodyB.GetLocalCenter));
   C1 :=  Subtract(Add(m_bodyB.m_sweep.c, rB), Add(m_bodyA.m_sweep.c, rA));
   {$ENDIF}

   C2 := m_bodyB.m_sweep.a - m_bodyA.m_sweep.a - m_referenceAngle;

   // Handle large detachment.
   {$IFDEF OP_OVERLOAD}
   positionError := C1.Length;
   {$ELSE}
   positionError := LengthVec(C1);
   {$ENDIF}
   angularError := Abs(C2);
   {if positionError > k_allowedStretch then
   begin
      iA *= 1.0;
      iB *= 1.0;
   end;}

   m_mass.col1.x := mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
   m_mass.col2.x := -rA.y * rA.x * iA - rB.y * rB.x * iB;
   m_mass.col3.x := -rA.y * iA - rB.y * iB;
   m_mass.col1.y := m_mass.col2.x;
   m_mass.col2.y := mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
   m_mass.col3.y := rA.x * iA + rB.x * iB;
   m_mass.col1.z := m_mass.col3.x;
   m_mass.col2.z := m_mass.col3.y;
   m_mass.col3.z := iA + iB;

   C.x := C1.x;
   C.y := C1.y;
   C.z := C2;

   {$IFDEF OP_OVERLOAD}
   impulse := m_mass.Solve33(-C);
   {$ELSE}
   impulse := Solve33(m_mass, Negative(C));
   {$ENDIF}

   P.x := impulse.x;
   P.y := impulse.y;

   {$IFDEF OP_OVERLOAD}
   m_bodyA.m_sweep.c.SubtractBy(mA * P);
   m_bodyB.m_sweep.c.AddBy(mB * P);
   {$ELSE}
   SubtractBy(m_bodyA.m_sweep.c, Multiply(P, mA));
   AddBy(m_bodyB.m_sweep.c, Multiply(P, mB));
   {$ENDIF}

   m_bodyA.m_sweep.a := m_bodyA.m_sweep.a - iA * (b2Cross(rA, P) + impulse.z);
   m_bodyB.m_sweep.a := m_bodyB.m_sweep.a + iB * (b2Cross(rB, P) + impulse.z);

   m_bodyA.SynchronizeTransform;
   m_bodyB.SynchronizeTransform;

   Result := (positionError <= b2_linearSlop) and (angularError <= b2_angularSlop);
end;

{ Tb2FixedJointDef }

constructor Tb2FixedJointDef.Create;
begin
   inherited;
	 JointType := e_fixedJoint;
end;

procedure Tb2FixedJointDef.Initialize(bodyA, bodyB: Tb2Body);
begin
   Self.bodyA := bodyA;
	 Self.bodyB := bodyB;
end;

{ Tb2FixedJoint }

constructor Tb2FixedJoint.Create(def: Tb2FixedJointDef);
begin
   inherited Create(def);
   // Get initial delta position and angle
   {$IFDEF OP_OVERLOAD}
   m_dp := b2MulT(m_bodyA.m_xf.R, m_bodyB.m_xf.position - m_bodyA.m_xf.position);
   {$ELSE}
   m_dp := b2MulT(m_bodyA.m_xf.R, Subtract(m_bodyB.m_xf.position, m_bodyA.m_xf.position));
   {$ENDIF}
   m_a := m_bodyB.GetAngle - m_bodyA.GetAngle;
   m_R0 := b2MulT(m_bodyA.m_xf.R, m_bodyB.m_xf.R);

   // Reset accumulated lambda
   m_lambda[0] := 0.0;
   m_lambda[1] := 0.0;
   m_lambda[2] := 0.0;
end;

procedure Tb2FixedJoint.CalculateMC;
var
   invM12, invI1, a, b, c, d, e, f, c1, c2, c3, den: Float;
begin
   UPhysics2DTypes.SinCos(m_bodyA.m_sweep.a, m_s, m_c);

   // Calculate vector A w1 := d/dt (R(a1) d)
   m_Ax := -m_s * m_d.x - m_c * m_d.y;
   m_Ay := m_c * m_d.x - m_s * m_d.y;

   // Calculate effective constraint mass: mC := (J M^-1 J^T)^-1
   invM12 := m_bodyA.m_invMass + m_bodyB.m_invMass;
   invI1 := m_bodyA.m_invI;
   a := invM12 + m_Ax * m_Ax * invI1;
   b := m_Ax * m_Ay * invI1;
   c := m_Ax * invI1;
   d := invM12 + m_Ay * m_Ay * invI1;
   e := m_Ay * invI1;
   f := m_bodyA.m_invI + invI1;
   c1 := d * f - e * e;
   c2 := c * e - b * f;
   c3 := b * e - c * d;
   den := a * c1 + b * c2 + c * c3;
   m_mc[0][0] := c1 / den;
   m_mc[1][0] := c2 / den;
   m_mc[2][0] := c3 / den;
   m_mc[0][1] := m_mc[1][0];
   m_mc[1][1] := (a * f - c * c ) / den;
   m_mc[2][1] := (b * c - a * e ) / den;
   m_mc[0][2] := m_mc[2][0];
   m_mc[1][2] := m_mc[2][1];
   m_mc[2][2] := (a * d - b * b ) / den;
end;

function Tb2FixedJoint.GetAnchorA: TVector2;
begin
	 // Return arbitrary position (we have to implement this abstract virtual function)
	 Result := m_bodyA.GetWorldCenter;
end;

function Tb2FixedJoint.GetAnchorB: TVector2;
begin
	 // Return arbitrary position (we have to implement this abstract virtual function)
	 Result := m_bodyB.GetWorldCenter;
end;

function Tb2FixedJoint.GetReactionForce(inv_dt: Float): TVector2;
begin
   {$IFDEF OP_OVERLOAD}
   Result := m_inv_dt * MakeVector(m_lambda[0], m_lambda[1]);
   {$ELSE}
   Result := Multiply(MakeVector(m_lambda[0], m_lambda[1]), m_inv_dt);
   {$ENDIF}
end;

function Tb2FixedJoint.GetReactionTorque(inv_dt: Float): Float;
begin
   Result := m_inv_dt * m_lambda[2];
end;

procedure Tb2FixedJoint.InitVelocityConstraints(const step: Tb2TimeStep);
begin
   // Store step
   m_inv_dt := step.inv_dt;

   // Get d for this step (transform from delta between positions to delta between center of masses)
   {$IFDEF OP_OVERLOAD}
   m_d := m_dp - m_bodyA.m_sweep.localCenter + b2Mul(m_R0, m_bodyB.m_sweep.localCenter);
   {$ELSE}
   m_d := Add(Subtract(m_dp, m_bodyA.m_sweep.localCenter), b2Mul(m_R0, m_bodyB.m_sweep.localCenter));
   {$ENDIF}

   // Calculate effective joint mass (stays constant during velocity solving)
   CalculateMC;

   if step.warmStarting then
   begin
      // Apply initial impulse
      with m_bodyA do
      begin
         m_linearVelocity.x := m_linearVelocity.x - m_invMass * m_lambda[0];
         m_linearVelocity.y := m_linearVelocity.y - m_invMass * m_lambda[1];
         m_angularVelocity := m_angularVelocity - m_invI * (m_lambda[0] * m_Ax + m_lambda[1] * m_Ay + m_lambda[2]);
      end;
      with m_bodyB do
      begin
         m_linearVelocity.x := m_linearVelocity.x + m_invMass * m_lambda[0];
         m_linearVelocity.y := m_linearVelocity.y + m_invMass * m_lambda[1];
         m_angularVelocity := m_angularVelocity + m_invI * m_lambda[2];
      end;
   end
   else
   begin
      // Reset accumulated lambda
      m_lambda[0] := 0.0;
      m_lambda[1] := 0.0;
      m_lambda[2] := 0.0;
   end;
end;

procedure Tb2FixedJoint.SolveVelocityConstraints(const step: Tb2TimeStep);
var
   i: Integer;
   Cdot, lambda: array[0..2] of Float;
begin
   // Assert that angle is still the same so the effective joint mass is still valid
   //assert(m_bodyA.m_sweep.a == m_a1);

   // Calculate Cdot
   Cdot[0] := m_bodyB.m_linearVelocity.x - m_bodyA.m_linearVelocity.x - m_Ax * m_bodyA.m_angularVelocity;
   Cdot[1] := m_bodyB.m_linearVelocity.y - m_bodyA.m_linearVelocity.y - m_Ay * m_bodyA.m_angularVelocity;
   Cdot[2] := m_bodyB.m_angularVelocity - m_bodyA.m_angularVelocity;

   // Calculate lambda
   for i := 0 to 2 do
      lambda[i] := -(m_mc[i][0] * Cdot[0] + m_mc[i][1] * Cdot[1] + m_mc[i][2] * Cdot[2]);

   // Apply impulse
   with m_bodyA do
   begin
      m_linearVelocity.x := m_linearVelocity.x - m_invMass * lambda[0];
      m_linearVelocity.y := m_linearVelocity.y - m_invMass * lambda[1];
      m_angularVelocity := m_angularVelocity - m_invI * (lambda[0] * m_Ax + lambda[1] * m_Ay + lambda[2]);
   end;
   with m_bodyB do
   begin
      m_linearVelocity.x := m_linearVelocity.x + m_invMass * lambda[0];
      m_linearVelocity.y := m_linearVelocity.y + m_invMass * lambda[1];
      m_angularVelocity := m_angularVelocity + m_invI * lambda[2];
   end;

   // Accumulate total lambda
   for i := 0 to 2 do
     m_lambda[i] := m_lambda[i] + lambda[i];
end;

function Tb2FixedJoint.SolvePositionConstraints(baumgarte: Float): Boolean;
var
   i: Integer;
   C, lambda: array[0..2] of Float;
begin
   // Recalculate effective constraint mass if angle changed enough
   if Abs(m_bodyA.m_sweep.a - m_a1) > 1e-3 then
     CalculateMC;

   // Calculate C
   C[0] := m_bodyB.m_sweep.c.x - m_bodyA.m_sweep.c.x - m_c * m_d.x + m_s * m_d.y;
   C[1] := m_bodyB.m_sweep.c.y - m_bodyA.m_sweep.c.y - m_s * m_d.x - m_c * m_d.y;
   C[2] := m_bodyB.m_sweep.a - m_a1 - m_a;

   // Calculate lambda
   for i := 0 to 2 do
     lambda[i] := -(m_mc[i][0] * C[0] + m_mc[i][1] * C[1] + m_mc[i][2] * C[2]);

   // Apply impulse
   with m_bodyA, m_sweep do
   begin
      c.x := c.x - m_invMass * lambda[0];
      c.y := c.y - m_invMass * lambda[1];
      a := a - m_invI * (lambda[0] * m_Ax + lambda[1] * m_Ay + lambda[2]);
   end;
   with m_bodyB, m_sweep do
   begin
      c.x := c.x + m_invMass * lambda[0];
      c.y := c.y + m_invMass * lambda[1];
      a := a + m_invI * lambda[2];
   end;

   // Push the changes to the transforms
   m_bodyA.SynchronizeTransform;
   m_bodyB.SynchronizeTransform;

   // Constraint is satisfied if all constraint equations are nearly zero
   Result := (Abs(C[0]) < b2_linearSlop) and (Abs(C[1]) < b2_linearSlop) and (Abs(C[2]) < b2_angularSlop);
end;

initialization
   b2_defaultFilter := Tb2ContactFilter.Create;
   b2_defaultListener := Tb2ContactListener.Create;

   b2_gjkCalls := 0;
   b2_gjkIters := 0;
   b2_gjkMaxIters := 0;

   _GrowableStack := Tb2GrowableStack.Create;
   world_query_wrapper := Tb2WorldQueryWrapper.Create;
   world_raycast_wrapper := Tb2WorldRayCastWrapper.Create;
   world_solve_island := Tb2Island.Create;
   world_solve_stack := TList.Create;
   contactsolver_positionsolver := Tb2PositionSolverManifold.Create;
   distance_simplex := Tb2Simplex.Create;
   toi_separation_fcn := Tb2SeparationFunction.Create;
   island_solve_contact_solver := Tb2ContactSolver.Create;
   static_edgeshape := Tb2EdgeShape.Create;
   static_polygonshape1 := Tb2PolygonShape.Create;
   static_polygonshape2 := Tb2PolygonShape.Create;

   {$IFDEF COMPUTE_PHYSICSTIME}
   QueryPerformanceFrequency(vCounterFrequency);
   {$ENDIF}

finalization
   b2_defaultFilter.Free;
   b2_defaultListener.Free;

   _GrowableStack.Free;
   world_query_wrapper.Free;
   world_raycast_wrapper.Free;
   world_solve_island.Free;
   world_solve_stack.Free;
   contactsolver_positionsolver.Free;
   distance_simplex.Free;
   toi_separation_fcn.Free;
   island_solve_contact_solver.Free;
   static_edgeshape.Free;
   static_polygonshape1.Free;
   static_polygonshape2.Free;

end.

