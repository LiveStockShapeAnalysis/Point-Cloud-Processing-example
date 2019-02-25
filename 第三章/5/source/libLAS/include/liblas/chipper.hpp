#ifndef LIBLAS_CHIPPER_H
#define LIBLAS_CHIPPER_H

#include <liblas/liblas.hpp>
#include <liblas/export.hpp>
#include <liblas/detail/opt_allocator.hpp>

#include <vector>

namespace liblas
{

namespace chipper
{

enum Direction
{
    DIR_X,
    DIR_Y,
    DIR_NONE
};

class LAS_DLL PtRef
{
public:
    double m_pos;
    uint32_t m_ptindex;
    uint32_t m_oindex;

    bool operator < (const PtRef& pt) const
        { return m_pos < pt.m_pos; }
};
typedef std::vector<PtRef, detail::opt_allocator<PtRef> > PtRefVec;

struct LAS_DLL RefList
{
public:
    PtRefVec *m_vec_p;
    Direction m_dir;

    RefList(Direction dir = DIR_NONE) : m_vec_p(NULL), m_dir(dir)
        {}
    ~RefList()
    {
        delete m_vec_p;
    }

    PtRefVec::size_type size() const
        { return m_vec_p->size(); }
    void reserve(PtRefVec::size_type n)
        { m_vec_p->reserve(n); }
    void resize(PtRefVec::size_type n)
        { m_vec_p->resize(n); }
    void push_back(const PtRef& ref)
        { m_vec_p->push_back(ref); }
    PtRefVec::iterator begin()
        { return m_vec_p->begin(); }
    PtRefVec::iterator end()
        { return m_vec_p->end(); }
    PtRef& operator[](uint32_t pos)
        { return (*m_vec_p)[pos]; }
    std::string Dir()
    {
        if (m_dir == DIR_X)
            return "X";
        else if (m_dir == DIR_Y)
            return "Y";
        else
            return "NONE";
    }
    void SortByOIndex(uint32_t left, uint32_t center,
        uint32_t right);
    void SetAllocator(detail::opt_allocator<PtRef> *alloc_p )
    {
        m_vec_p = new PtRefVec( *alloc_p );
    }
};

class LAS_DLL Chipper;

class LAS_DLL Block
{
    friend class Chipper;

private:
    RefList *m_list_p;
    uint32_t m_left;
    uint32_t m_right;
    liblas::Bounds<double> m_bounds;

public:
    std::vector<uint32_t> GetIDs() const; 
    Bounds<double> const& GetBounds() const
        {return m_bounds;} 
    void SetBounds(liblas::Bounds<double> const& bounds)
        {m_bounds = bounds;}
};

// Options that can be used to modify the behavior of the chipper.
class LAS_DLL Options
{
public:
    Options() : m_threshold( 1000 ), m_use_sort( false ),
       m_use_maps( false )
    {}

    // Maximum number of pointer per output block.
    uint32_t m_threshold;
    // If true, use sorting instead of copying to reduce memory.
    bool m_use_sort;
    // If true, use memory mapped files instead of main memory
    bool m_use_maps;
    // Map file to use if m_use_maps is true.
    std::string m_map_file;
};

class LAS_DLL Chipper
{
public:
    Chipper(Reader *reader, Options *options );
    Chipper(Reader *reader, uint32_t max_partition_size) :
        m_reader(reader), m_xvec(DIR_X), m_yvec(DIR_Y), m_spare(DIR_NONE)
    {
        m_options.m_threshold = max_partition_size;
    }

    void Chip();
    std::vector<Block>::size_type GetBlockCount()
        { return m_blocks.size(); }
    const Block& GetBlock(std::vector<Block>::size_type i)
        { return m_blocks[i]; }

private:
    int Allocate();
    int Load();
    void Partition(uint32_t size);
    void Split(RefList& xvec, RefList& yvec, RefList& spare);
    void DecideSplit(RefList& v1, RefList& v2, RefList& spare,
        uint32_t left, uint32_t right);
    void RearrangeNarrow(RefList& wide, RefList& narrow, RefList& spare,
        uint32_t left, uint32_t center, uint32_t right);
    void Split(RefList& wide, RefList& narrow, RefList& spare,
        uint32_t left, uint32_t right);
    void FinalSplit(RefList& wide, RefList& narrow,
        uint32_t pleft, uint32_t pcenter);
    void Emit(RefList& wide, uint32_t widemin, uint32_t widemax,
        RefList& narrow, uint32_t narrowmin, uint32_t narrowmax);

    Reader *m_reader;
    std::vector<Block> m_blocks;
    std::vector<uint32_t> m_partitions;
    // Note, order is important here, as the allocator must be destroyed
    // after the RefLists.
    boost::shared_ptr<detail::opt_allocator<PtRef> > m_allocator;
    RefList m_xvec;
    RefList m_yvec;
    RefList m_spare;
    Options m_options;
};

} // namespace chipper

} // namespace liblas

#endif
