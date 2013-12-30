/// Small color object for each particle
/// Constructor with four elements: r (red), g (green), b (blue), and a (opacity).
/// Each element can be specified 0 to 255.
function b2ParticleColor(r, g, b, a)
{
	if (r instanceof b2Color)
	{
		this.r = (255 * r.r);
		this.g = (255 * r.g);
		this.b = (255 * r.b);
		this.a = 255;
	}
	else if (typeof(r) !== 'undefined')
	{
		this.r = r;
		this.g = g;
		this.b = b;
		this.a = a;
	}
	else
		this.r = this.g = this.b = this.a = 0;
}

b2ParticleColor.prototype =
{
	/// True when all four color elements equal 0. When true, no memory is used for particle color.
	///
	IsZero: function()
	{
		return !this.r && !this.g && !this.b && !this.a;
	},

	/// Used internally to convert the value of b2Color.
	///
	GetColor: function()
	{
		return new b2Color(
			1.0 / 255 * this.r,
			1.0 / 255 * this.g,
			1.0 / 255 * this.b);
	},

	/// Sets color for current object using the four elements described above.
	///
	Set: function(r_, g_, b_, a_)
	{
		if (r_ instanceof b2Color)
		{
			this.r = (255 * r_.r);
			this.g = (255 * r_.g);
			this.b = (255 * r_.b);
			this.a = 255;
		}
		else
		{
			this.r = r_;
			this.g = g_;
			this.b = b_;
			this.a = a_;
		}
	},

	Assign: function(pc)
	{
		this.r = pc.r;
		this.g = pc.g;
		this.b = pc.b;
		this.a = pc.a;
	},

	Clone: function()
	{
		return new b2ParticleColor(this.r, this.g, this.b, this.a);
	}
};

b2ParticleColor.zero = new b2ParticleColor();

/// A particle definition holds all the data needed to construct a particle.
/// You can safely re-use these definitions.
function b2ParticleDef()
{
	/// Specifies the type of particle. A particle may be more than one type.
	/// Multiple types are chained by logical sums, for example:
	/// pd.flags = b2_elasticParticle | b2_viscousParticle
	this.flags = 0;

	/// The world position of the particle.
	this.position = new b2Vec2();

	/// The linear velocity of the particle in world co-ordinates.
	this.velocity = new b2Vec2();

	/// The color of the particle.
	this.color = new b2ParticleColor();

	/// Use this to store application-specific body data.
	this.userData = null;
}

/// The particle type. Can be combined with | operator.
/// Zero means liquid.
b2ParticleDef.b2_waterParticle =       0;
b2ParticleDef.b2_zombieParticle =      1 << 1; // removed after next step
b2ParticleDef.b2_wallParticle =        1 << 2; // zero velocity
b2ParticleDef.b2_springParticle =      1 << 3; // with restitution from stretching
b2ParticleDef.b2_elasticParticle =     1 << 4; // with restitution from deformation
b2ParticleDef.b2_viscousParticle =     1 << 5; // with viscosity
b2ParticleDef.b2_powderParticle =      1 << 6; // without isotropic pressure
b2ParticleDef.b2_tensileParticle =     1 << 7; // with surface tension
b2ParticleDef.b2_colorMixingParticle = 1 << 8; // mixing color between contacting particles
b2ParticleDef.b2_destructionListener = 1 << 9; // call b2DestructionListener on destruction