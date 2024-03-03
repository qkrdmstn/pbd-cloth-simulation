#include "Hash.h";

Hash::Hash()
{

}

Hash::Hash(double _spacing, int _numParticles, int _numOfTri)
{
	spacing = _spacing;
	numParticles = _numParticles;
	numOfTri = _numOfTri;
	tableSize = 5 * _numParticles; //table size�� ��ƼŬ ���� 5���
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

int Hash::intCoord(double coord) //double ���� int ������ ���� (cell Pos(int)�� ��ȯ)
{
	return floor(coord / spacing);
}

int Hash::hashCoords(int xi, int yi, int zi) //hash function���� �迭 index get
{
	int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481); //hash function
	return abs(h) % tableSize;
}

int Hash::hashPos(vec3 pos) //������ ��ġ�� �Է¹޾Ƽ� int������ ������ ��, hash function�� ������ index ��ȯ
{
	return hashCoords(intCoord(pos.x()), intCoord(pos.y()), intCoord(pos.z()));
}

void Hash::create(vector<vec3> _pos)
{
	cellStart.assign(cellStart.size(), 0); //0���� ä���
	cellEntries.assign(cellEntries.size(), 0);

	for (int i = 0; i < numParticles; i++) //Count
	{
		int h = hashPos(_pos[i]); //�ش� ������ �ش��ϴ� cell Index�� �迭�� ���ϱ�
		cellStart[h]++;
	}

	int start = 0;
	for (int i = 0; i < tableSize; i++) //Partial sums
	{
		start += cellStart[i];
		cellStart[i] = start;
	}
	cellStart[tableSize] = start; //������ ĭ ä���

	for (int i = 0; i < numParticles; i++) //Fill in
	{
		int h = hashPos(_pos[i]);
		cellStart[h]--;
		cellEntries[cellStart[h]] = i;
	}
}

void Hash::query(vector<vec3> _pos, int index, double maxDist)
{
	int x0 = intCoord(_pos[index].x() - maxDist); //���� �������� - maxDist ��ŭ�� ��ġ
	int y0 = intCoord(_pos[index].y() - maxDist);
	int z0 = intCoord(_pos[index].z() - maxDist);

	int x1 = intCoord(_pos[index].x() + maxDist); //���� �������� + maxDist ��ŭ�� ��ġ
	int y1 = intCoord(_pos[index].y() + maxDist);
	int z1 = intCoord(_pos[index].z() + maxDist);

	querySize = 0;

	for (int xi = x0; xi <= x1; xi++) //x, y, z ���� 1�� ���ϸ鼭 maxDist ���� �ִ� ���� ��ȸ
	{
		for (int yi = y0; yi <= y1; yi++)
		{
			for (int zi = z0; zi <= z1; zi++)
			{
				int h = hashCoords(xi, yi, zi); //�ش� ��ġ�� index ��ȯ�ް�
				int start = cellStart[h];
				int end = cellStart[h + 1];

				for (int i = start; i < end; i++) //�ش� index�� �ִ� �������� ��� queryIds�� ����
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
			if (id1 >= id0) //�ߺ� ����
				continue;

			double dist2 = (_pos[id0] - _pos[id1]).lengthSquared();
			if (dist2 > maxDist2)
				continue;

			if (num >= adjIds.size()) //adjIds size�� �۴ٸ�, resize
				adjIds.resize(2 * num);

			adjIds[num] = id1;
			num++;
		}
	}
	firstAdjId[numParticles] = num; //������ index ä���	
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
			if (_tri[id0].point[0] == id1 || _tri[id0].point[1] == id1 || _tri[id0].point[2] == id1) //�ڽ��� ������ ����
				continue;
			if (num >= TadjIds.size()) //adjIds size�� �۴ٸ�, resize
				TadjIds.resize(2 * num);

			TadjIds[num] = id1;
			num++;
		}
	}
	TfirstAdjId[numOfTri] = num; //������ index ä���	
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

	for (int xi = x0; xi <= x1; xi++) //x, y, z ���� 1�� ���ϸ鼭 BoundingBox ���� �ִ� ���� ��ȸ
	{
		for (int yi = y0; yi <= y1; yi++)
		{
			for (int zi = z0; zi <= z1; zi++)
			{
				int h = hashCoords(xi, yi, zi); //�ش� ��ġ�� index ��ȯ�ް�
				int start = cellStart[h];
				int end = cellStart[h + 1];

				for (int i = start; i < end; i++) //�ش� index�� �ִ� �������� ��� queryIds�� ����
				{
					TqueryIds[TquerySize] = cellEntries[i];
					TquerySize++;
				}
			}
		}
	}
}
