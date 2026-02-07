#include "gltf.h"

#include <string>
#include <string_view>
#include <cctype>
#include <stdexcept>
#include <iostream>
#include <array>
#include <optional>
#include <vector>
#include <ostream>
#include <iosfwd>
#include <fstream>
#include <sstream>
#include <map>
#include <variant>
#include <assert.h>

#include "vector.h"
#include "matrix.h"

namespace {

	struct BufferView
	{
		size_t buffer;
		size_t byteLength;
		size_t byteOffset;
		size_t target;
		size_t byteStride;
	};

	struct Accessor
	{
		size_t bufferView;
		size_t byteOffset;
		size_t componentType;
		size_t count;
		Vector3 max;
		Vector3 min;
		std::string type;
	};

	enum class TokenType {
		LBrace,		// {
		RBrace,		// }
		LBracket,	// [
		RBracket,	// ]
		Colon,		// :
		Comma,		// ,
		String,
		Number,
		Bool,
		End
	};

	struct Token {
		TokenType type;
		std::string_view text;
		size_t line;
		size_t column;
	};

	class Lexer
	{
	public:
		explicit Lexer(std::string_view input) : _input(input)
		{
			double x = 0;
		}

		Token next()
		{
			skipWhitespace();

			if (_pos >= _input.size())
				return { TokenType::End, "", _line, _column };

			char c = _input[_pos];

			switch (c)
			{
			case '{': return simple(TokenType::LBrace);
			case '}': return simple(TokenType::RBrace);
			case '[': return simple(TokenType::LBracket);
			case ']': return simple(TokenType::RBracket);
			case ':': return simple(TokenType::Colon);
			case ',': return simple(TokenType::Comma);
			case '"': return string();
			default:
				if (std::isdigit(c) || c == '-')
					return number();
				else
					return boolean();
			}
			error("unexpect character");
			return { TokenType::End, "", _line, _column };
		}

	private:
		void skipWhitespace()
		{
			while (_pos < _input.size()) {
				char c = _input[_pos];
				if (c == ' ' || c == '\t' || c == '\r') {
					advance();
				}
				else if (c == '\n') {
					_pos++;
					_line++;
					_column = 1;
				}
				else {
					break;
				}
			}
		}

		void advance() {
			_pos++;
			_column++;
		}


		Token simple(TokenType type)
		{
			Token t{ type, _input.substr(_pos, 1), _line, _column };
			advance();
			return t;
		}

		Token string()
		{
			size_t startLine = _line;
			size_t startCol = _column;

			size_t start = ++_pos;
			_column++;

			while (_pos < _input.size() && _input[_pos] != '"') {
				if (_input[_pos] == '\n')
				{
					error("newline in string");
				}
				advance();
			}

			if (_pos >= _input.size())
			{
				error("unterminated string");
			}

			auto text = _input.substr(start, _pos - start);
			advance();

			return { TokenType::String, text, startLine, startCol };
		}

		Token boolean()
		{
			size_t startLine = _line;
			size_t startCol = _column;
			size_t start = _pos;

			while (_pos < _input.size() && std::isalpha(_input[_pos]))
				advance();

			return {
				TokenType::Bool,
				_input.substr(start, _pos - start),
				startLine,
				startCol
			};
		}

		Token number()
		{
			size_t startLine = _line;
			size_t startCol = _column;
			size_t start = _pos;

			if (_input[_pos] == '-')
				advance();

			while (_pos < _input.size() && std::isdigit(_input[_pos]))
				advance();

			if (_pos < _input.size() && _input[_pos] == '.') {
				advance();
				while (_pos < _input.size() && std::isdigit(_input[_pos]))
					advance();
			}

			if (_pos < _input.size() && (_input[_pos] == 'e' || _input[_pos] == 'E')) {
				advance();
				if (_input[_pos] == '+' || _input[_pos] == '-')
					advance();
				while (_pos < _input.size() && std::isdigit(_input[_pos]))
					advance();
			}

			return {
				TokenType::Number,
				_input.substr(start, _pos - start),
				startLine,
				startCol
			};
		}

		void error(const char* msg)
		{
			throw std::runtime_error(
				std::string(msg) +
				" at line " + std::to_string(_line) +
				", column " + std::to_string(_column)
			);
		}

	private:
		std::string_view _input;
		size_t _pos = 0;
		size_t _line = 1;
		size_t _column = 1;
	};


	class Parser {
		struct Node {
			std::string name;
			std::optional<int> camera;
			std::optional<int> mesh;
			Vector3 translation{ 0,0,0 };
			Vector3 scale{ 1,1,1 };
			Vector4 rotation{ 0,0,0,1 };
		};

		struct Camera {
			std::string name;
			float aspectRatio = 1.0f;
			float yfov = 0.7f;
			float znear = 0.1f;
			float zfar = 100.0f;
		};

		struct Scene {
			std::string name;
			std::vector<int> nodes;
		};

		struct Material
		{
			std::string name;
			Vector3 emissiveFactor;
			float emissiveStrength;
			Vector4 baseColorFactor;
			float metallicFactor;
			float roughnessFactor;
		};

		struct Mesh
		{
			struct Primitive {
				size_t material;
				size_t indices;
				std::map<std::string, size_t> attributes;
			};
			std::string name;
			std::vector<Primitive> primitives;
		};


		struct Buffer
		{
			size_t byteLength;
			std::string uri;
		};

	public:
		struct SceneFile {
			int defaultScene = 0;
			std::vector<Scene> scenes;
			std::vector<Node> nodes;
			std::vector<Camera> cameras;
			std::vector<Material> materials;
			std::vector<Mesh> meshes;
			std::vector<Accessor> accessors;
			std::vector<BufferView> bufferViews;
			std::vector<Buffer> buffers;
		};

	public:
		Parser(Lexer& lexer) : _lexer(lexer)
		{
			advance();
		}

		SceneFile parseSceneFile() {
			SceneFile file;
			expect(TokenType::LBrace);

			while (!match(TokenType::RBrace)) {
				std::string key = consumeString();
				expect(TokenType::Colon);

				if (key == "scene")
					file.defaultScene = consumeInt();
				else if (key == "scenes")
					parseScenes(file.scenes);
				else if (key == "nodes")
					parseNodes(file.nodes);
				else if (key == "cameras")
					parseCameras(file.cameras);
				else if (key == "materials")
					parseMaterials(file.materials);
				else if (key == "meshes")
					parseMeshes(file.meshes);
				else if (key == "accessors")
					parseAccessors(file.accessors);
				else if (key == "bufferViews")
					parseBufferViews(file.bufferViews);
				else if (key == "buffers")
					parseBuffers(file.buffers);
				else
					skipValue();

				match(TokenType::Comma);
			}
			return file;
		}

	private:
		void advance()
		{
			_token = _lexer.next();
		}

		bool match(TokenType type)
		{
			if (_token.type == type)
			{
				advance();
				return true;
			}
			return false;
		}

		bool check(TokenType type) const
		{
			return _token.type == type;
		}

		void expect(TokenType type)
		{
			if (!match(type))
			{
				error("unexpected token");
			}
		}

		void skipValue()
		{
			if (match(TokenType::String) || match(TokenType::Number))
				return;

			if (match(TokenType::LBrace))
			{
				while (!match(TokenType::RBrace)) {
					skipValue(); // key
					expect(TokenType::Colon);
					skipValue(); // value
					match(TokenType::Comma);
				}
				return;
			}

			if (match(TokenType::LBracket))
			{
				while (!match(TokenType::RBracket)) {
					skipValue();
					match(TokenType::Comma);
				}
				return;
			}

			error("invalid value");
		}

		std::string consumeString()
		{
			if (_token.type != TokenType::String)
				error("expected string");
			std::string s(_token.text);
			advance();
			return s;
		}

		int consumeInt()
		{
			if (_token.type != TokenType::Number)
				error("expected number");
			int v = std::stoi(std::string(_token.text));
			advance();
			return v;
		}

		float consumeFloat()
		{
			if (_token.type != TokenType::Number)
				error("expected number");
			float v = std::stof(std::string(_token.text));
			advance();
			return v;
		}

		class Element {
		public:

			using type = std::variant<
				bool,
				int,
				float,
				Vector3,
				Vector4,
				std::string,
				Element,
				std::vector<Element>,
				std::vector<float>
			>;

			Element() = default;


			void add(const std::string& name, type value) {
				_attributes[name] = std::move(value);
			}

			type* get(const std::string& name) {
				auto it = _attributes.find(name);
				if (it != _attributes.end()) {
					return &(it->second);
				}
				return nullptr;
			}

			bool has(const std::string& name) const {
				return _attributes.find(name) != _attributes.end();
			}

			template<typename T>
			T getAs(const std::string& name, T defaultValue = T()) const {
				auto it = _attributes.find(name);
				if (it != _attributes.end()) {
					if (auto valPtr = std::get_if<T>(&it->second)) {
						return *valPtr;
					}
				}
				return defaultValue;
			}

			std::vector<std::string> names() const
			{
				std::vector<std::string> v;
				for (const auto& attr : _attributes)
				{
					v.push_back(attr.first);
				}
				return v;
			}

		private:
			std::map<std::string, type> _attributes;
		};

		Element::type parseArray()
		{
			expect(TokenType::LBracket);
			if (check(TokenType::Number))
			{
				std::vector<float> values;
				while (!match(TokenType::RBracket))
				{
					values.push_back(consumeFloat());
					match(TokenType::Comma);
				}

				if (values.size() == 3) {
					return Vector3{ values[0], values[1], values[2] };
				}
				else if (values.size() == 4) {
					return Vector4{ values[0], values[1], values[2], values[3] };
				}

				return values;
			}
			else
			{
				std::vector<Element> values;
				while (!match(TokenType::RBracket))
				{
					values.push_back(parseElement());
					match(TokenType::Comma);
				}
				return values;
			}
		}

		Element::type parseValue()
		{
			switch (_token.type) {
			case TokenType::Number:
				return consumeFloat();

			case TokenType::String:
				return consumeString();

			case TokenType::LBrace:
				return parseElement();

			case TokenType::LBracket:
				return parseArray();

			case TokenType::Bool:
				return consumeString() == "true";

			default:
				throw std::runtime_error("Unexpected token type");
			}
		}

		Element parseElement()
		{
			expect(TokenType::LBrace);
			Element el;
			while (!match(TokenType::RBrace)) {
				std::string key = consumeString();
				expect(TokenType::Colon);

				el.add(key, parseValue());

				match(TokenType::Comma);
			}
			return el;
		}

		void parseCameras(std::vector<Camera>& cameras)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				Camera c;
				c.name = el.getAs<std::string>("name", "None");
				c.aspectRatio = el.getAs<Element>("perspective").getAs<float>("aspectRatio", 1.0);
				c.yfov = el.getAs<Element>("perspective").getAs<float>("yfov", 1.0);
				c.znear = el.getAs<Element>("perspective").getAs<float>("znear", 1.0);
				c.zfar = el.getAs<Element>("perspective").getAs<float>("zfar", 1.0);

				cameras.push_back(c);
			}
		}

		void parseMaterials(std::vector<Material>& materials)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				Material m;
				m.name = el.getAs<std::string>("name", "None");
				m.emissiveFactor = el.getAs<Vector3>("emissiveFactor", Vector3(0.0f, 0.0f, 0.0f));
				m.baseColorFactor = el.getAs<Element>("pbrMetallicRoughness").getAs<Vector4>("baseColorFactor", Vector4(1.0f, 1.0f, 1.0f, 1.0f));
				m.metallicFactor = el.getAs<Element>("pbrMetallicRoughness").getAs<float>("metallicFactor", 1.0f);
				m.roughnessFactor = el.getAs<Element>("pbrMetallicRoughness").getAs<float>("roughnessFactor", 1.0f);
				m.emissiveStrength = el.getAs<Element>("extensions").getAs<Element>("KHR_materials_emissive_strength").getAs<float>("emissiveStrength", 1.0);

				materials.push_back(m);
			}
		}

		void parseMeshes(std::vector<Mesh>& meshes)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				Mesh m;
				m.name = el.getAs<std::string>("name", "None");
				for (const auto& pr : el.getAs<std::vector<Element>>("primitives"))
				{
					Mesh::Primitive p;
					p.indices = (int)pr.getAs<float>("indices");
					p.material = (int)pr.getAs<float>("material");

					auto attrs = pr.getAs<Element>("attributes");

					for (const auto& name : attrs.names())
					{
						p.attributes.emplace(name, (int)attrs.getAs<float>(name.c_str()));
					}

					m.primitives.push_back(p);
				}
				meshes.push_back(m);
			}
		}

		void parseNodes(std::vector<Node>& nodes)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				Node n;
				n.name = el.getAs<std::string>("name");
				if (el.has("camera"))
				{
					n.camera = (int)el.getAs<float>("camera");
				}
				if (el.has("mesh"))
				{
					n.mesh = (int)el.getAs<float>("mesh");
				}
				n.rotation = el.getAs<Vector4>("rotation", Vector4(0.0f, 0.0f, 0.0f, 1.0f ));
				n.translation = el.getAs<Vector3>("translation");
				n.scale = el.getAs<Vector3>("scale", Vector3(1.0f, 1.0f, 1.0f));

				nodes.push_back(n);
			}
		}

		void parseScenes(std::vector<Scene>& scenes)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				Scene s;
				s.name = el.getAs<std::string>("name", "None");
				for (const auto& n : el.getAs<std::vector<float>>("nodes"))
				{
					s.nodes.push_back((int)n);
				}
				scenes.push_back(s);
			}
		}

		void parseAccessors(std::vector<Accessor>& accessors)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				Accessor accessor;
				accessor.bufferView = (int)el.getAs<float>("bufferView");
				accessor.componentType = (int)el.getAs<float>("componentType");
				accessor.count = (int)el.getAs<float>("count");
				accessor.max = el.getAs<Vector3>("max");
				accessor.min = el.getAs<Vector3>("min");
				accessor.type = el.getAs<std::string>("type");
				accessor.byteOffset = (int)el.getAs<float>("byteOffset");

				accessors.push_back(accessor);
			}
		}

		void parseBufferViews(std::vector<BufferView>& bufferViews)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				BufferView bf;
				bf.buffer = (int)el.getAs<float>("buffer");
				bf.byteLength = (int)el.getAs<float>("byteLength");
				bf.byteOffset = (int)el.getAs<float>("byteOffset");
				bf.target = (int)el.getAs<float>("target");
				bf.byteStride = (int)el.getAs<float>("byteStride", 0.0f);
				bufferViews.push_back(bf);
			}
		}

		void parseBuffers(std::vector<Buffer>& buffers)
		{
			std::vector<Element> elements = std::get<std::vector<Element>>(parseValue());
			for (const auto& el : elements)
			{
				Buffer b;
				b.byteLength = (int)el.getAs<float>("byteLength");
				b.uri = el.getAs<std::string>("uri");
				buffers.push_back(b);
			}
		}

		void error(const char* msg)
		{

		}

	private:
		Lexer& _lexer;
		Token _token;
	};


	enum class GltfComponentType : uint32_t
	{
		BYTE = 5120,
		UNSIGNED_BYTE = 5121,
		SHORT = 5122,
		UNSIGNED_SHORT = 5123,
		UNSIGNED_INT = 5125,
		FLOAT = 5126
	};

	void writeGltfComponent(GltfComponentType type, const void* in, void* out)
	{
		switch (type)
		{
		case GltfComponentType::BYTE:			*reinterpret_cast<int8_t*>(out) = *reinterpret_cast<const int8_t*>(in);	break;
		case GltfComponentType::UNSIGNED_BYTE:	*reinterpret_cast<uint8_t*>(out) = *reinterpret_cast<const uint8_t*>(in);	break;
		case GltfComponentType::SHORT:			*reinterpret_cast<int16_t*>(out) = *reinterpret_cast<const int16_t*>(in);	break;
		case GltfComponentType::UNSIGNED_SHORT:	*reinterpret_cast<uint16_t*>(out) = *reinterpret_cast<const uint16_t*>(in);	break;
		case GltfComponentType::UNSIGNED_INT:	*reinterpret_cast<uint32_t*>(out) = *reinterpret_cast<const uint32_t*>(in);	break;
		case GltfComponentType::FLOAT:			*reinterpret_cast<float*>(out) = *reinterpret_cast<const float*>(in);	break;
		default:
			assert(false && "Unknown glTF component type");
		}
	}

	enum class GltfType
	{
		SCALAR,
		VEC2,
		VEC3,
		VEC4,
		MAT2,
		MAT3,
		MAT4
	};

	class GltfBin
	{
	public:
		GltfBin() = default;

		bool loadFromFile(const std::string& path)
		{
			std::ifstream file(path, std::ios::binary | std::ios::ate);
			if (!file)
				return false;

			const std::streamsize fileSize = file.tellg();
			file.seekg(0, std::ios::beg);

			data_.resize(static_cast<size_t>(fileSize));
			if (!file.read(reinterpret_cast<char*>(data_.data()), fileSize))
				return false;

			return true;

		}

		const uint8_t* data() const { return data_.data(); }
		size_t size() const { return data_.size(); }

		template<typename T>
		const T* read(size_t byteOffset) const
		{
			assert(byteOffset + sizeof(T) <= data_.size());
			return reinterpret_cast<const T*>(data_.data() + byteOffset);
		}


		template<typename T>
		const T* readArray(size_t byteOffset, size_t count) const
		{
			assert(byteOffset + sizeof(T) * count <= data_.size());
			return reinterpret_cast<const T*>(data_.data() + byteOffset);
		}

	private:
		std::vector<uint8_t> data_;
	};

	struct AccessorView
	{
		const uint8_t* data = nullptr;
		size_t stride = 0;
		size_t elementSize = 0;
		size_t count = 0;
	};

	inline size_t componentSize(GltfComponentType t)
	{
		switch (t)
		{
		case GltfComponentType::BYTE:
		case GltfComponentType::UNSIGNED_BYTE:  return 1;
		case GltfComponentType::SHORT:
		case GltfComponentType::UNSIGNED_SHORT: return 2;
		case GltfComponentType::UNSIGNED_INT:
		case GltfComponentType::FLOAT:          return 4;
		}
		return 0;
	}

	inline size_t typeComponentCount(const std::string& t)
	{
		if (t == "SCALAR")	return 1;
		if (t == "VEC2")	return 2;
		if (t == "VEC3")	return 3;
		if (t == "VEC4")	return 4;
		if (t == "MAT2")	return 4;
		if (t == "MAT3")	return 9;
		if (t == "MAT4")	return 16;

		return 0;
	}

	AccessorView readAccessor(
		const GltfBin& bin,
		const BufferView& bufferView,
		const Accessor& accessor)
	{
		AccessorView out{};

		const size_t compSize = componentSize(GltfComponentType(accessor.componentType));
		const size_t compCount = typeComponentCount(accessor.type);
		const size_t elemSize = compSize * compCount;

		const size_t stride = bufferView.byteStride != 0 ? bufferView.byteStride : elemSize;

		const size_t baseOffset =
			bufferView.byteOffset +
			accessor.byteOffset;


		assert(baseOffset + stride * (accessor.count - 1) + elemSize
			<= bufferView.byteOffset + bufferView.byteLength);

		out.data = bin.data() + baseOffset;
		out.stride = stride;
		out.elementSize = elemSize;
		out.count = accessor.count;

		return out;
	}

}

namespace gltf {

	bool parse(const char* fileName, Scene& scene)
	{
		std::ifstream file(fileName);
		if (!file.is_open()) {
			std::cerr << "Can`t open file " << fileName << std::endl;
			return false;
		}

		std::stringstream buffer;
		buffer << file.rdbuf();

		std::string s = buffer.str();
		Lexer lexer(s);
		Parser parser(lexer);
		Parser::SceneFile gltfScene = parser.parseSceneFile();


		GltfBin bin;
		bin.loadFromFile("../scenes/" + gltfScene.buffers[0].uri);

		for (const auto& node : gltfScene.nodes)
		{
			if (node.mesh.has_value())
			{
				std::string name = gltfScene.meshes[*node.mesh].name;
				for (const auto& prim : gltfScene.meshes[*node.mesh].primitives)
				{

					std::vector<int> indices;
					std::vector<Vector3> positions;
					size_t matIndex = prim.material;
					{
						const auto acc = gltfScene.accessors[prim.indices];
						const auto view = gltfScene.bufferViews[acc.bufferView];
						const auto idx = readAccessor(bin, view, acc);

						for (size_t i = 0; i < idx.count; ++i)
						{
							const uint8_t* ptr = idx.data + i * idx.elementSize;

							uint32_t index = 0;
							writeGltfComponent(GltfComponentType(acc.componentType), ptr, &index);

							indices.push_back(index);
						}
					}
					{
						for (const auto& [semantic, accessorIndex] : prim.attributes)
						{
							const Accessor& acc = gltfScene.accessors[accessorIndex];

							const BufferView& view = gltfScene.bufferViews[acc.bufferView];

							AccessorView a = readAccessor(bin, view, acc);

							if (semantic == "POSITION")
							{
								const size_t compSize = componentSize(GltfComponentType(acc.componentType));
								for (size_t i = 0; i < a.count; ++i)
								{
									Vector3 v;
									const uint8_t* ptr = a.data + i * a.elementSize;
									writeGltfComponent(GltfComponentType(acc.componentType), ptr + 0 * compSize, &v[0]);
									writeGltfComponent(GltfComponentType(acc.componentType), ptr + 1 * compSize, &v[1]);
									writeGltfComponent(GltfComponentType(acc.componentType), ptr + 2 * compSize, &v[2]);

									positions.push_back(v);
								}
							}
							else if (semantic == "NORMAL") {} // VEC3 float 
							else if (semantic == "TEXCOORD_0") {}// VEC2 float
							else if (semantic == "TANGENT") {} // VEC4 float
						}
					}
					Matrix4 nodeWorld = computeLocalMatrix(node.translation, node.scale, Quaternion({ node.rotation.x(), node.rotation.y(), node.rotation.z(), node.rotation.w() }));

					std::vector<math::Triangle> triangles;
					for (int i = 0; i < indices.size(); )
					{
						Vector3 p0 = transformPoint(nodeWorld, positions[indices[i + 0]]);
						Vector3 p1 = transformPoint(nodeWorld, positions[indices[i + 1]]);
						Vector3 p2 = transformPoint(nodeWorld, positions[indices[i + 2]]);
						triangles.push_back({ p0, p1, p2, matIndex });
						i += 3;
					}

					scene.addNode(node.name, triangles);
				}

			}
			else if (node.camera.has_value())
			{
				
				Matrix4 local = computeLocalMatrix(node.translation, node.scale, Quaternion({ node.rotation.x(), node.rotation.y(), node.rotation.z(), node.rotation.w() }));
				Camera c;
				c.fov = gltfScene.cameras[*node.camera].yfov;
				c.up = transformVector(local, Vector3(0, 1, 0));
				c.aspectRatio = gltfScene.cameras[*node.camera].aspectRatio;

				
				c.pos = transformPoint(local, Vector3(0.0f, 0.0f, 0.0f));
				Vector3 forward = transformVector(local, Vector3(0, 0, -1));
				c.target = c.pos + forward;
				scene.setCamera(c);
			}
		}

		for (const auto& m : gltfScene.materials)
		{
			Material mat;
			mat.albedo = Vector3(m.baseColorFactor.x(), m.baseColorFactor.y(), m.baseColorFactor.z());
			mat.emission = m.emissiveFactor * m.emissiveStrength;
			mat.metallic = m.metallicFactor;
			mat.roughness = m.roughnessFactor;
			scene.addMaterial(mat);
		}

		return false;
	}
}