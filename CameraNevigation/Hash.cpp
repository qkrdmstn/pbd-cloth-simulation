#include "Hash.h";

Hash::Hash()
{

}

Hash::Hash(double _spacing, int _numParticles, int _numOfTri)
{
	spacing = _spacing;
	numParticles = _numParticles;
	numOfTri = _numOfTri;
	tableSize = 5 * _numParticles; //table size를 파티클 수의 5배로
	querySize = 0;
	TquerySize = 0;

	cellStart.resize(tableSize + 1);
	cellEntries.resize(numParticles);

	queryIds.resize(3 * numParticles);
	firstAdjId.resize(numParticles + 1);
	adjIds.resize(10 * numParticles);

	TqueryIds.resize(3 * numOfTri);
	TfirstAdjId.resize(numOfTri + 1);
	TadjIds.resize(10 * numOfTri);
}
Hash::~Hash()
{

}

int Hash::intCoord(double coord) //double 값을 int 값으로 내림 (cell Pos(int)로 변환)
{
	return floor(coord / spacing);
}

int Hash::hashCoords(int xi, int yi, int zi) //hash function으로 배열 index get
{
	int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481); //hash function
	return abs(h) % tableSize;
}

int Hash::hashPos(vec3 pos) //정점의 위치를 입력받아서 int형으로 내림한 뒤, hash function을 돌려서 index 반환
{
	return hashCoords(intCoord(pos.x()), intCoord(pos.y()), intCoord(pos.z()));
}

void Hash::create(vector<vec3> _pos)
{
	cellStart.assign(cellStart.size(), 0); //0으로 채우기
	cellEntries.assign(cellEntries.size(), 0);

	for (int i = 0; i < numParticles; i++) //Count
	{
		int h = hashPos(_pos[i]); //해당 정점에 해당하는 cell Index를 배열에 더하기
		cellStart[h]++;
	}

	int start = 0;
	for (int i = 0; i < tableSize; i++) //Partial sums
	{
		start += cellStart[i];
		cellStart[i] = start;
	}
	cellStart[tableSize] = start; //마지막 칸 채우기

	for (int i = 0; i < numParticles; i++) //Fill in
	{
		int h = hashPos(_pos[i]);
		cellStart[h]--;
		cellEntries[cellStart[h]] = i;
	}
}

void Hash::query(vector<vec3> _pos, int index, double maxDist)
{
	int x0 = intCoord(_pos[index].x() - maxDist); //현재 정점에서 - maxDist 만큼의 위치
	int y0 = intCoord(_pos[index].y() - maxDist);
	int z0 = intCoord(_pos[index].z() - maxDist);

	int x1 = intCoord(_pos[index].x() + maxDist); //현재 정점에서 + maxDist 만큼의 위치
	int y1 = intCoord(_pos[index].y() + maxDist);
	int z1 = intCoord(_pos[index].z() + maxDist);

	querySize = 0;

	for (int xi = x0; xi <= x1; xi++) //x, y, z 값을 1씩 더하면서 maxDist 내에 있는 셀을 순회
	{
		for (int yi = y0; yi <= y1; yi++)
		{
			for (int zi = z0; zi <= z1; zi++)
			{
				int h = hashCoords(xi, yi, zi); //해당 위치의 index 반환받고
				int start = cellStart[h];
				int end = cellStart[h + 1];

				for (int i = start; i < end; i++) //해당 index에 있는 정점들을 모두 queryIds에 대입
				{
					queryIds[querySize] = cellEntries[i];
					querySize++;
				}
			}
		}
	}
}

void Hash::queryAll(vector<vec3> _pos, double maxDist)
{
	int num = 0;
	double maxDist2 = maxDist * maxDist;

	for (int i = 0; i < numParticles; i++)
	{
		int id0 = i;
		firstAdjId[id0] = num;
		query(_pos, id0, maxDist);

		for (int j = 0; j < querySize; j++)
		{
			int id1 = queryIds[j];
			if (id1 >= id0) //중복 방지
				continue;

			double dist2 = (_pos[id0] - _pos[id1]).lengthSquared();
			if (dist2 > maxDist2)
				continue;

			if (num >= adjIds.size()) //adjIds size가 작다면, resize
				adjIds.resize(2 * num);

			adjIds[num] = id1;
			num++;
		}
	}
	firstAdjId[numParticles] = num; //마지막 index 채우기	
}


void Hash::triQueryAll(vector<Triangle> _tri)
{
	int num = 0;
	for (int i = 0; i < numOfTri; i++)
	{
		int id0 = i;
		TfirstAdjId[id0] = num;
		triQuery(_tri, id0);

		for (int j = 0; j < TquerySize; j++)
		{
			int id1 = TqueryIds[j];
			if (_tri[id0].point[0] == id1 || _tri[id0].point[1] == id1 || _tri[id0].point[2] == id1) //자신의 꼭짓점 제외
				continue;
			if (num >= TadjIds.size()) //adjIds size가 작다면, resize
				TadjIds.resize(2 * num);

			TadjIds[num] = id1;
			num++;
		}
	}
	TfirstAdjId[numOfTri] = num; //마지막 index 채우기	
}

void Hash::triQuery(vector<Triangle> _tri, int index)
{
	int x0 = intCoord(_tri[index].minPos.x()) - 1;
	int y0 = intCoord(_tri[index].minPos.y()) - 1;
	int z0 = intCoord(_tri[index].minPos.z()) - 1;

	int x1 = intCoord(_tri[index].maxPos.x()) + 1;
	int y1 = intCoord(_tri[index].maxPos.y()) + 1;
	int z1 = intCoord(_tri[index].maxPos.z()) + 1;

	TquerySize = 0;

	for (int xi = x0; xi <= x1; xi++) //x, y, z 값을 1씩 더하면서 BoundingBox 내에 있는 셀을 순회
	{
		for (int yi = y0; yi <= y1; yi++)
		{
			for (int zi = z0; zi <= z1; zi++)
			{
				int h = hashCoords(xi, yi, zi); //해당 위치의 index 반환받고
				int start = cellStart[h];
				int end = cellStart[h + 1];

				for (int i = start; i < end; i++) //해당 index에 있는 정점들을 모두 queryIds에 대입
				{
					TqueryIds[TquerySize] = cellEntries[i];
					TquerySize++;
				}
			}
		}
	}
}
